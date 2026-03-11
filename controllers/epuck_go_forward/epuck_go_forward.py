from controller import Robot, Motor, Camera, RangeFinder

TIME_STEP = 64
MAX_SPEED = 6.28

# tune this so robot moves about one tile (0.25m)
FORWARD_STEPS_ONE_TILE = 60

# range finder wall threshold
RANGEFINDER_FRONT_THRESHOLD = 0.23


TOTAL_TILES = 36

# ------------------ create robot ------------------
robot = Robot()

# ------------------ motors ------------------
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# ------------------ camera ------------------
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# ------------------ range finder ------------------
rangeFinder = robot.getDevice('range-finder')
rangeFinder.enable(TIME_STEP)
RF_WIDTH = rangeFinder.getWidth()
RF_HEIGHT = rangeFinder.getHeight()

# ------------------ maze state ------------------
# 0 = north, 1 = east, 2 = south, 3 = west
heading = 0
current_tile = (0, 0)

visited = set()
visited.add(current_tile)

path_stack = [current_tile]
unexplored_dirs = {}

goal_tile = None

dir_names = ['N', 'E', 'S', 'W']

# ------------------ helper functions ------------------
def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    robot.step(TIME_STEP)
    robot.step(TIME_STEP)

def turn_left_90():
    global heading
    stop()
    leftMotor.setVelocity(-0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
    for _ in range(12):
        robot.step(TIME_STEP)
    stop()
    heading = (heading - 1) % 4

def turn_right_90():
    global heading
    stop()
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    for _ in range(12):
        robot.step(TIME_STEP)
    stop()
    heading = (heading + 1) % 4

def turn_180():
    global heading
    stop()
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    for _ in range(24):
        robot.step(TIME_STEP)
    stop()
    heading = (heading + 2) % 4

def turn_to_direction(target_direction):
    diff = (target_direction - heading) % 4

    if diff == 0:
        return
    elif diff == 1:
        turn_right_90()
    elif diff == 2:
        turn_180()
    elif diff == 3:
        turn_left_90()

# Move forward one tile - will have to test optimal 'FORWARD_STEPS_ONE_TILE'
def move_forward_one_tile():
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
    for _ in range(FORWARD_STEPS_ONE_TILE):
        robot.step(TIME_STEP)
    stop()

    # if too close to a wall, reverse until we reach 0.13
    d = front_distance()
    if d <= 0.11:
        print(f"TOO CLOSE ({d:.3f}), backing up")
        leftMotor.setVelocity(-0.3 * MAX_SPEED)
        rightMotor.setVelocity(-0.3 * MAX_SPEED)
        while True:
            robot.step(TIME_STEP)
            d = front_distance()
            if d >= 0.13:
                break
        stop()

def sees_green():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    r_total = g_total = b_total = 0
    count = width * height

    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)

            r_total += r
            g_total += g
            b_total += b

    r_avg = r_total / count
    g_avg = g_total / count
    b_avg = b_total / count

    return g_avg > r_avg * 1.2 and g_avg > b_avg * 1.2


# Get teh minimum
def front_distance():
    image = rangeFinder.getRangeImage()
    cx = RF_WIDTH // 2
    cy = RF_HEIGHT // 2

    values = []

    for y in range(max(0, cy - 1), min(RF_HEIGHT, cy + 2)):
        for x in range(max(0, cx - 2), min(RF_WIDTH, cx + 3)):
            d = RangeFinder.rangeImageGetDepth(image, RF_WIDTH, x, y)
            values.append(d)

    if not values:
        return 999.0

    center_min = min(values)
    print("Range:", center_min)
    return center_min

def front_blocked_by_rangefinder():
    return front_distance() <= RANGEFINDER_FRONT_THRESHOLD

# Get the tile coordinates in the given direction from the current tile (0=north, 1=east, 2=south, 3=west)
def tile_in_direction(tile, direction):
    x, y = tile
    if direction == 0:
        return (x, y - 1)
    elif direction == 1:
        return (x + 1, y)
    elif direction == 2:
        return (x, y + 1)
    else:
        return (x - 1, y)

# Check if the path in the given direction is open using the range finder, and if not, reverse until it's clear
def direction_open(direction):
    turn_to_direction(direction)
    robot.step(TIME_STEP)
    robot.step(TIME_STEP)

    # if too close after scanning, reverse until 0.125 to help recenter the epuck
    d = front_distance()
    if d <= 0.11:
        print(f"TOO CLOSE after scan ({d:.3f}), reversing...")
        leftMotor.setVelocity(-0.3 * MAX_SPEED)
        rightMotor.setVelocity(-0.3 * MAX_SPEED)
        while True:
            robot.step(TIME_STEP)
            d = front_distance()
            if d >= 0.125:
                break
        stop()

    d = front_distance()
    is_open = d > RANGEFINDER_FRONT_THRESHOLD
    if is_open:
        print(f"OPEN PATH detected: {dir_names[direction]} from {current_tile} (range={d})")
    return is_open

# Update unexplored directions for the current tile based on rangefinder and visited tiles
def update_unexplored_dirs(tile):
    if tile not in unexplored_dirs:
        unexplored_dirs[tile] = set()

    original_heading = heading

    forward_dir = original_heading
    left_dir = (original_heading - 1) % 4
    right_dir = (original_heading + 1) % 4

    for direction in [forward_dir, left_dir, right_dir]:
        neighbor = tile_in_direction(tile, direction)

        if neighbor in visited:
            print(f"SKIP visited direction: {dir_names[direction]} from {tile} -> {neighbor}")
            continue

        if direction_open(direction):
            if direction not in unexplored_dirs[tile]:
                unexplored_dirs[tile].add(direction)
                print(f"STORE unexplored direction: {dir_names[direction]} from {tile}")

    turn_to_direction(original_heading)

# ------------------ main loop ------------------
while robot.step(TIME_STEP) != -1:

    # record the green wall/tile of green wall but do not stop unless all 36 tiles have been explored
    if sees_green():
        if goal_tile is None:
            goal_tile = current_tile
            print(f"GREEN FOUND at {goal_tile}")

        if len(visited) >= TOTAL_TILES:
            print("GREEN FOUND and all 36 tiles found")
            leftMotor.setVelocity(0.5 * MAX_SPEED)
            rightMotor.setVelocity(0.5 * MAX_SPEED)
            while robot.step(TIME_STEP) != -1:
                pass
            break
        else:
            print(f"GREEN FOUND AT {current_tile}, only {len(visited)}/{TOTAL_TILES} tiles explored so must continue")

    update_unexplored_dirs(current_tile)

    moved = False

    # prefer forward, then left, then right
    priority = [heading, (heading - 1) % 4, (heading + 1) % 4]

    # extra prints for deferred choices
    available_now = unexplored_dirs.get(current_tile, set()).copy()
    if len(available_now) > 1:
        print(f"Multiple open paths stored at {current_tile}: {[dir_names[d] for d in available_now]}")

    for direction in priority:
        if direction in unexplored_dirs.get(current_tile, set()):
            # print skipped stored alternatives
            for other_dir in sorted(unexplored_dirs[current_tile].copy()):
                if other_dir != direction:
                    print(f"DEFER direction: {dir_names[other_dir]} from {current_tile} for later exploration")

            next_tile = tile_in_direction(current_tile, direction)
            unexplored_dirs[current_tile].remove(direction)

            turn_to_direction(direction)
            move_forward_one_tile()

            current_tile = next_tile
            visited.add(current_tile)
            path_stack.append(current_tile)

            print(f"MOVE -> {current_tile} ({dir_names[heading]})")
            print(f"Visited tiles: {len(visited)}/{TOTAL_TILES}")

            moved = True
            break

    if moved:
        continue

    # Backtracking logic --------------
    if len(path_stack) <= 1:
        print("Done exploring")
        stop()
        break

    path_stack.pop()
    prev_tile = path_stack[-1]

    # Determine direction to backtrack
    dx = prev_tile[0] - current_tile[0]
    dy = prev_tile[1] - current_tile[1]

    # Map (dx, dy) to direction: (0, -1) = north(0), (1, 0) = east(1), (0, 1) = south(2), (-1, 0) = west(3)
    if dy == -1:
        back_dir = 0
    elif dx == 1:
        back_dir = 1
    elif dy == 1:
        back_dir = 2
    else:
        back_dir = 3

    print(f"BACKTRACKING: {current_tile} -> {prev_tile} ({dir_names[back_dir]})")
    turn_to_direction(back_dir)
    move_forward_one_tile()
    current_tile = prev_tile
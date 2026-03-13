from controller import Robot, DistanceSensor, Motor, Camera

TIME_STEP = 64
MAX_SPEED = 6.28
THRESHOLD = 250.0

# ------------------ create robot ------------------
robot = Robot()

# ------------------ distance sensors ------------------
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
ps = []
for name in psNames:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# ------------------ motors ------------------
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# ------------------ camera ------------------
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# ------------------ helper functions ------------------

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

    # green dominates
    return g_avg > r_avg * 1.2 and g_avg > b_avg * 1.2

def turn_left():
    leftMotor.setVelocity(0.1 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
    for _ in range(6):
        robot.step(TIME_STEP)
    
def turn_right():
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.1 * MAX_SPEED)
    
def go_straight():
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)

# ------------------ main loop ------------------

while robot.step(TIME_STEP) != -1:

    psValues = [sensor.getValue() for sensor in ps]

    # openness (lower = more open)
    right_space = psValues[2]   # right = ps2
    front_space = psValues[0]

    if sees_green():
            print("Green wall detected")
            go_straight()
    elif front_space > 90: 
        print("obstacle in front")
        turn_left()
    elif right_space > 90:
        print("obstacle to the right")
        go_straight()
    else: 
        turn_right()
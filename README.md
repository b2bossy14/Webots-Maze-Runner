## SETUP
1. Add a **RangeFinder** to the e-puck's **turret slot**
2. Make sure the **controller** is set to **epuck_go_forward**

## Mapping and Timed Run
1. The robot will complete the mapping of the maze and return to the starting tile
2. The robot will flash all its top LEDs red and body LEDs green to indicate the start of the timed run

## Backup
1. If there are issues with the mapping run, set the **controller** to **epuck_backup_right_wall_follower** and run the simulation again
2. The timer can be started immediately upon the start of the simulation when running the backup
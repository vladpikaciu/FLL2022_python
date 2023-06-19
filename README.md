# FLL2022 Python
Python programs used by the All for One team from Balti during the 2022 FLL championship.

The program contains:
  - 1 Main file called brazilia2022.py that executes all the other script at the press of the center button on the [ev3 robot](https://pybricks.com/ev3-micropython/examples/robot_educator_basic.html).
  - 4 files that contain the program for every run of the robot on the match table.
  - 1 Pikachu Image that is shown on the screen when the program is executed. (for fun)

# Short summary about the code:
-We used the micropython version of the pybricks library to communicate with the operating system of the robot.
The program contains many new functions, we even decided it would be beneficial to have a separate file called functions.py that was used to invoke functions in other files.
-As a personal goal, I wanted to tinker with the ev3 brick and try to find ways to make it more custom and unique, and finally I decided to change it's operating system. Our robot now runs on the ev3dev version which is a Debian Linux-based system. 
This enabled me to connect through SSH and make changes to it's system (which was impossible on the ev3 original OS). I almost succeded to run a **1.8 minecraft server** on our Lego robot.

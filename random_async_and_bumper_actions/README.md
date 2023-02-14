# Simple Action Example
This directory contains a custom ROS2 package which, when run demonstrates using actions to:

1. Undock robot
2. Drive 1 meter
3. Grab pose
4. Loop to randomly turn and drive .75 meters
5. Return to dock

It also demonstrates using the keyboard to call functions which start the robot actions and how to handle
the bumper subscription.

## Example Setup
1. `cd` into example directory.
2. Run `colcon build`.
3. Start new terminal session.
4. `cd` into example directory and source setup with `source install/setup.sh`.
5. `cd` into `random_async_and_bumper_actions/src/random_walk/random_walk`.
6. Run code with `python3 randWalk.py`
7. Use `r` key to start actions.

## Dependencies
* install Pynput library for keyboard input with `pip install pynput`
* _**Git submodules:**_ the irobot_create_msgs package is included here as a submodule. This means that when you pull this repository `src/irobot_create_msgs` may be an empty folder. To fix this run `git submodule update --init --recursive` or simply clone irobot_create_msgs to the `src/` directory with:

        git clone -b humble https://github.com/iRobotEducation/irobot_create_msgs.git

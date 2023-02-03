# datachecker

Datachecker for Bag Files. Checks topics defined in an input yaml file to ensure the messages from these topics are being published close to an expected frequency. Will notify in output for unexpected increases/decreases in frequency rate and skipped messages based on message id. To run (1) configure settings.yaml to your desired bag file/topics, build project, and run ./datachecker. 

## Input Configuration
Settings.yaml is configured with (1) bag_file - a string argument on what bag file to use, (2) topics - a list of topics defined by their name, expected rate, and std dev in the expected rate

## Output Format

When you output, you will get 2 files (1) log file and (2) synopsis file. The log file will show all instances of the types of errors noted above with timestamp, topic info, error type. The synopsis file will contain a brief overall summary, such as time elapsed in bag file. 

## Build

1. Add the project directory to `ROS_PACKAGE_PATH`:
    ```
    export ROS_PACKAGE_PATH=MYDIRECTORY:$ROS_PACKAGE_PATH
    ```
    (Replace `MYDIRECTORY` with the actual directory)
    You can also add this to your `~/.bashrc` file so that you don't have to do
    this every time you open a new terminal.
1. Build the program:
    ```
    make
    ```
    Optionally, to compile on all cores (make sure you have sufficient RAM!)
    ```
    make -j
    ```
1. Do **not** run `cmake`, `catkin_make`, `rosbuild`.

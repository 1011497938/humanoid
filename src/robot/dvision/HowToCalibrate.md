How to calibrate camera.
===

1. calibrate camera intrinsic
2. calibrate camera extrinsic
3. update config


Before everything start
===
```
$ roslaunch dconfig dconfig.launch
```
So that we can have a roscore running and configs loaded

Intrinsic
===

1. Take images of chessboard
    
    ```
    $ rl dvision capture.launch
    ```
    Press `c` to capture pictures, all the image files will be located at current working directory.

2. use opencv to calibrate intrinsics
     
     ```
     $ rosrun dvision calib <image directory>
     ```
    Just press space, wait for some moment when cv is calibrating.
    The result will be showed on the console. Copy them to dconfig/dvision/camera.yml


Extrinsic
===

1. take images
    
    ```
    $ roslaunch dlaunch getImg.launch
    ```

    Run this on the robot, and take many pictures with robot standing at the center of the field, facing to the goal.

2. mark points
    
    ```
    rl dcalibrator dcalibrator.launch

    ```
    Generate a text file, make sure the intrinsic parameters we calibrated before are correctly loaded.
    

3. run matlab
    
    run `main.m`

Note
===

Remember to run `roslaunch dconfig dconfig.launch`

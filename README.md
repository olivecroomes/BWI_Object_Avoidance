How to run our program:

1. turn on the laptop

2. turn on the segway base

3. on the laptop, run 
      $ roslaunch segbot_bringup segbot_hokuyo.launch

4. on the computer, in TAB1 of the terminal, run
      $ roslaunch objectAvoidance object_avoidance.launch --screen
      
5. on the computer, in TAB2 of the terminal, also run
      $ rosrun objectAvoidance avoidance

6. finally, go to TAB1, click inside, and run the teleop-commands as normal. The robot should avoid objects.

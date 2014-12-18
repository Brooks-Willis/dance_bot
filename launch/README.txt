Run record.launch to record a video. Instructions should be printed in the terminal after a few second for taking the recording. Make sure to keep the elbow, wrist, and shoulder visible at all time - it will not record data ppoints where one is missing. The arm radius is currently set to 1 - this controls how the person's arm is scaled to the robot arm, and should be changed as appropriate.

roslaunch dance_bot record.launch

If you want to record mulitple takes, the instead of killing it with Ctrl-C, send any string to the topic /kill_sig

rostopic pub /kill_sig std_msgs/String "stop!" 

---

Then, run the dmp server and then learn_a_funky_move.py. Pass in the name of the funky move to learn and a value of K (one of the gains) and num_bases (number of basis functions) from the command line after running the node to learn and save DMPs.

roslaunch dmp dmp.launch
rosrun dance_bot learn_a_funky_move.py [path to dance_bot]/path_data
OR 
roslaunch dance_bot learn.launch

---

To actually run the arm, run arm_node.py, plan_a_funky_move.py, and request_a_funky_move.py (with the arm connected and the dmp server running). Bust a move by entering the name (no quotes), start point (as a list of integers) and end point (as a list of integers).

roslaunch dmp dmp.launch
rosrun dance_bot bust_a_funky_move.py
rosrun dance_bot plan_a_funky_move.py /home/rboy/catkin_ws/src/dance_bot/path_data
rosrun dance_bot request_a_funky_move.py
OR
roslaunch dance_bot run.launch



Depends on: csv, json, pandas, dmp, numpy, dependencies outlined in the fiducial tracking folder
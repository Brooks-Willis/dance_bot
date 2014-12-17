With Kyle and Allie's fiducial tracking code running, run transform_person_to_arm.py (adjust the path to save the data at and the person's arm radius appropriately).

rosrun dance_bot transform_person_to_arm.py

To make sure the file is written completely and records only the data you want, send any string on the topic /kill_sig when you want to stop recording. (this won't kill the node, just close the file)

rostopic pub /kill_sig std_msgs/String "stop!" 

---

Then, run the dmp server and then learn_a_funky_move.py with the path specified above and a value of K (one of the gains) and num_bases (number of basis functions)

roslaunch dmp dmp.launch
rosrun dance_bot learn_a_funky_move.py /home/rboy/catkin_ws/src/dance_bot/path_data/test1.csv 500 5

---

To actually run the arm, run arm_node.py and plan_a_funky_move.py (with the arm connected and the dmp server running). Specific moves can then be commanded by making a PlanRequest, as deomnstrated in the script request_a_funky_move.py.

roslaunch dmp dmp.launch
rosrun dance_bot arm_node.py
rosrun dance_bot plan_a_funky_move.py /home/rboy/catkin_ws/src/dance_bot/path_data

rosrun dance_bot request_a_funky_move.py



Depends on: csv, json, pandas, dmp, numpy, whatever Allie and Kyle's stuff depends on
# Final Project – How We Ran It

Team: Sandeep Arsule, Harshith Pidur Kuppusamy (Group 29)

These are the steps we actually ran; follow them to replay our checks.

Context and constraints:
- We had to work headless from macOS; our Windows setup failed and EC2 GPU hours were too costly, so we stayed on the TIAGo container.
- Because of time pressure we focused on command-line verification and logging rather than polishing the UI or adding more tests.
- The notes below capture exactly what we ran so can be reproduced from the terminal.

## Common setup (each terminal)
```
docker exec -it -w /home/user/exchange/ssy236_arsule tiago bash
source /knowrob_ws/devel/setup.bash
catkin_make
source devel/setup.bash
```

## 1) Gazebo (headless is fine)
```
roslaunch world_percept_assig4 gazebo_ssy236.launch gzclient:=false
# wait until: rostopic echo -n 1 /clock
```

## 2) Full stack (perception + learning + reasoning + control)
```
roslaunch world_percept_assig4 final_project.launch \
  queries_root:=$(rospack find world_percept_assig4) \
  target_name:=bookshelf \
  use_direct_perception:=true \
  percept_use_ground_truth:=true
```
We used ground truth here so it’s easy to see the target; we can flip those flags to false for pure sensors.
Observed stack highlights from our run:
```
[ WARN] Target object [bookshelf] not found
[ INFO] Got new object: Bookshelf
New class created: Bookshelf
[ WARN] new instance in knowledge base: ...#Bookshelf_1
[ INFO] Observed bookshelf on bookshelf (predicted: )
...
[ INFO] Got new object: CupRed
New class created: CupRed
[ WARN] new instance in knowledge base: ...#CupRed_2
```

## 3) Optional: poke the bookshelf in
If we don’t want to drive the robot close, just shove the bookshelf into the learner:
```
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetModelState
from world_percept_assig4.srv import UpdateObjectList, UpdateObjectListRequest
rospy.init_node('poke_bookshelf')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
upd = rospy.ServiceProxy('/update_object_list', UpdateObjectList)
state = get_state('bookshelf','')
print(upd(UpdateObjectListRequest(object_name='bookshelf', object_pose=state.pose)))
PY
```

## 4) Reasoning quick checks
```
rosservice call /assert_knowledge "object_name: 'CupRed'"

rosrun rosprolog rosprolog
register_ros_package(world_percept_assig4).
ensure_loaded('/home/user/exchange/ssy236_arsule/src/autonomous-reasoning-robot/prolog/init.pl').
observe_instance('Bottle','table',1.0).
preferred_surface('Bottle', S).
search_target('Bottle', S, C).
needs_confirmation('Bottle').
decision_to_revisit('Bottle', S).
halt.
```
Observed output from our run:
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosrun rosprolog rosprolog
Welcome to SWI-Prolog (threaded, 64 bits, version 7.6.4)
...
?- register_ros_package(world_percept_assig4).
true.
?- ensure_loaded('/home/user/exchange/ssy236_arsule/src/autonomous-reasoning-robot/prolog/init.pl').
true.
?- observe_instance('Bottle','table',1.0).
true.
?- preferred_surface('Bottle', S).
true.
?- search_target('Bottle', S, C).
C = 1.
?- needs_confirmation('Bottle').
false.
?- decision_to_revisit('Bottle', S).
false.
?- halt.
```
Also:
```
rosservice call /get_scene_object_list "object_name: 'bookshelf'"
```

## 5) Learning + online tool + evaluation
```
rostopic echo -n 1 /learning/confusion_json
rosservice call /report_learning_metrics
rosrun world_percept_assig4 learning_metrics.py   # prints sklearn confusion/accuracy if installed
```
Observed output from our run:
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rostopic echo -n 1 /learning/confusion_json
data: "{\"bookshelf\":{\"untrained\":1}}"

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosservice call /report_learning_metrics
success: True
message: "{\"bookshelf\":{\"untrained\":1}}"

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosrun world_percept_assig4 learning_metrics.py
[INFO] [1768512896.974855, 3825.984000]: Confusion labels: ['bookshelf', 'untrained']
[INFO] [1768512896.975981, 3825.984000]: Counts matrix: [[0, 1], [0, 0]]
[WARN] [1768512896.977193, 3825.985000]: sklearn not available; showing raw counts only.
```

## 6) Control loop + pointing skill
```
rosservice call /compute_target_twist "target_name: 'bookshelf'"
rostopic echo -n 5 /mobile_base_controller/cmd_vel
rostopic echo -n 5 /learning/obstacle_distance
rostopic echo -n 1 /arm_controller/command
rostopic echo -n 1 /head_controller/command
```
Observed output from our run:
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosservice call /compute_target_twist "target_name: 'bookshelf'"
cmd_vel:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: -1.7459857265126216e-05}
success: True
message: "Command computed"

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rostopic echo -n 5 /mobile_base_controller/cmd_vel
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -1.731559431382936e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -1.724745325427025e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -1.7353162457053196e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -1.7353163216802342e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: -1.7323033146811857e-05}

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rostopic echo -n 5 /learning/obstacle_distance
WARNING: no messages received and simulated time is active.
Is /clock being published?

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rostopic echo -n 1 /arm_controller/command
joint_names: [arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint, arm_7_joint]
points:
  - positions: [0.25, -1.0, -0.2, 1.5, 0.0, -0.6, 0.0]
    time_from_start: {secs: 2, nsecs: 500000000}

root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rostopic echo -n 1 /head_controller/command
joint_names: [head_1_joint, head_2_joint]
points:
  - positions: [0.0007550168041108116, -0.25]
    time_from_start: {secs: 1, nsecs: 0}
```

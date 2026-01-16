# Final Project

Team: Sandeep Arsule, Harshith Pidur Kuppusamy (Group 29)

These are the steps we followed for headless gazebo test run.

Context and constraints:
- We had to work headless from macOS; our Windows setup failed and EC2 GPU hours were too costly, so we stayed on the TIAGo container.
- We ran out of time from other exam and thesis prepp, though this is not excuse but we needed do this mostly from terminal.
- The notes below capture exactly what we ran so can be reproduced from the terminal.
- Also we kept old package name from assignment 4

## Common setup (each terminal)
```
docker exec -it -w /home/user/exchange/ssy236_arsule tiago bash
source /knowrob_ws/devel/setup.bash
catkin_make
source devel/setup.bash
```

## 1) Gazebo (headless one)
```
roslaunch final_project gazebo_ssy236.launch gzclient:=false
# wait until: rostopic echo -n 1 /clock
```
Observed output from our run (headless, so RViz fails):
```
[ WARN] Unable to update multi-DOF joint 'odom_to_base': base_footprint not yet available
qt.qpa.xcb: could not connect to display ... (RViz dies without X)
rviz process has died (headless; ignore)
Controller Spawner: Loaded/Started controllers (imu, gripper, force_torque, torso, head, arm, mobile_base)
MoveGroup using OMPL; arm tuck succeeded
```

## 2) Full stack (perception + learning + reasoning + control)
```
roslaunch final_project final_project.launch use_direct_perception:=true percept_use_ground_truth:=true target_name:=table
```
We used ground truth here so it’s easy to see the target; we can flip those flags to false for pure sensors. Headless notes: expect “target not found” warnings until you add/see it, and RViz will fail without X. Once you seed/see `table` you’ll get `Got new object: Table` and warnings stop.

## 3) Seed targets (table and bottle)
We poked `table` into the learning node so services can find it:
```
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetModelState
from final_project.srv import UpdateObjectList, UpdateObjectListRequest
rospy.init_node('poke_table')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
upd = rospy.ServiceProxy('/update_object_list', UpdateObjectList)
st = get_state('table','')
print(upd(UpdateObjectListRequest(object_name='table', object_pose=st.pose)))
PY
```
Seed the bottle (`coke_can` in the kitchen world):
```
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import GetModelState
from final_project.srv import UpdateObjectList, UpdateObjectListRequest
rospy.init_node('poke_coke_can')
get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
upd = rospy.ServiceProxy('/update_object_list', UpdateObjectList)
st = get_state('coke_can','')
print(upd(UpdateObjectListRequest(object_name='coke_can', object_pose=st.pose)))
PY
rosservice call /get_scene_object_list "object_name: 'coke_can'"
```

## 4) Reasoning quick checks
```
rosrun rosprolog rosprolog
register_ros_package(final_project).
ensure_loaded('/home/user/exchange/ssy236_arsule/src/autonomous-reasoning-robot/prolog/init.pl').
observe_instance('Bottle','table',1.0).
preferred_surface('Bottle', S).
search_target('Bottle', S, C).
halt.
```
Observed output:
```
?- register_ros_package(final_project).
true.
?- ensure_loaded('/home/user/exchange/ssy236_arsule/src/autonomous-reasoning-robot/prolog/init.pl').
true.
?- observe_instance('Bottle','table',1.0).
true.
?- preferred_surface('Bottle', S).
true.  % S = table (binding not printed)
?- search_target('Bottle', S, C).
C = 1.
?- halt.
```

## 5) Learning + online tool + evaluation
```
rostopic echo -n 1 /learning/confusion_json
rosservice call /report_learning_metrics
```
Observed output (after seeding `table`/`coke_can`):
```
data: "{\"table\":{\"untrained\":2}}"
success: True
message: "{\"table\":{\"untrained\":2}}"
```

## 6) Control loop + pointing skill
```
rosservice call /compute_target_twist "target_name: 'table'"
rostopic echo -n 5 /mobile_base_controller/cmd_vel
```
Observed output (after seeding `table` and `coke_can`):
```
cmd_vel:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 5.185526687674876e-05}
success: True
message: "Command computed"

linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 5.004823599341976e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 4.991077656566374e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 4.969337201469638e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 4.9198288624871526e-05}
---
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 4.9140668572718835e-05}
```

## What we accomplished (tasks and subtasks)
- Brought up Gazebo headlessly and confirmed simulated time via `/clock`.
- Built and launched the full stack (perception, learning, reasoning, control) with ground-truth perception for reproducibility.
- Seeded and registered `table` (target) and `coke_can` (bottle) via the service and confirmed with `/get_scene_object_list`.
- Ran reasoning checks in `rosprolog` (load `init.pl`, observe Bottle on table, query preferred surface/search target).
- Collected learning metrics (confusion JSON, metrics service; sklearn not available so raw counts).
- Exercised control: computed target twist for `table` and observed the base cmd_vel stream.
- Noted `/learning/obstacle_distance` stays empty when `/scan` is not publishing.

## Mapping to FP tasks (headless run)
- FP.T01 Reasoning: Prolog rules to store sightings and pick a best surface (`observe_instance`, `preferred_surface`, `search_target`, `needs_confirmation`, `decision_to_revisit`); facts pushed via `update_object_list`.
- FP.T02 Learning: learning node keeps add-one smoothed surface counts; we use the running stack as the online tool and report confusion via `/learning/confusion_json` and `/report_learning_metrics` (raw counts without sklearn).
- FP.T03 Robotics: full perception–action loop with direct Gazebo perception, object updates, computed twist to the target (`table`), and base/arm/head commands for motion/pointing.

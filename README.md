# SSY236 – Final Project
Group 29  
Sandeep Arsule, Harshith Pidur Kuppusamy

This package implements the final project requirements (perception, reasoning, control and learning). The old `map_generator_node` has been removed; a new `learning_node` now connects perception, reasoning and control, performs online learning, and exposes evaluation metrics.

## Run the full stack
```
docker exec -it -w /home/user/ tiago bash

cd /home/user/exchange/ssy236_arsule
source /knowrob_ws/devel/setup.bash
catkin_make
source devel/setup.bash

# 1) Simulator
roslaunch world_percept_assig4 gazebo_ssy236.launch

# 2) Perception + learning + reasoning + control (new stack)
roslaunch world_percept_assig4 final_project.launch \
  queries_root:=$(rospack find world_percept_assig4) \
  target_name:=bookshelf \
  use_direct_perception:=false \
  percept_use_ground_truth:=false

# 2b) (Optional) Teleop to stimulate perception
# Use key_teleop to move Tiago so the vision/laser perception in learning_node can see objects.
# Example: rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mobile_base_controller/cmd_vel

# 3) Load previously asserted knowledge (optional warm start)
rosservice call /load_knowledge "start: 1"

# 4) (Optional) Live ML metrics using scikit-learn if installed
rosrun world_percept_assig4 learning_metrics.py
```

## Key interfaces (all provided by `learning_node`)
- `update_object_list` (`world_percept_assig4/UpdateObjectList`): used by `percept_node` and vision to push new detections (poses go straight into the learner and ontology).
- `get_scene_object_list` (`world_percept_assig4/GetSceneObjectList`): queried by `tiago_control_node`; falls back to learned Prolog decisions when a target has not been observed yet.
- `report_learning_metrics` (`std_srvs/Trigger`): returns the live confusion matrix for the learned Naive Bayes model.
- Topic `learning/obstacle_distance` (`std_msgs/Float32`): front-laser based safety distance that gates the controller.

## What changed for the Final Project
- Removed `map_generator_node`; the controller now talks only to `learning_node`, which in turn calls `reasoning_node` to assert knowledge.
- Added online learning over object/support relations (Dirichlet-smoothed counts + confusion matrix evaluation). Exposes JSON metrics on `learning/confusion_json` for a scikit-learn consumer (`scripts/learning_metrics.py`) to compute accuracy and confusion matrices (fulfills “online tool” requirement). If sklearn is not present, the script logs raw counts only.
- Added OpenCV-based vision hook (red blob detection) and laser-based perception loop; both feed `learning_node` so the perception stack uses real sensors rather than world state. Ground-truth perception (`/gazebo/model_states`) is disabled by default (`use_ground_truth:=false` in `final_project.launch`); enable it only for debugging.
- Added a pointing skill in `tiago_control_node` (arm + head trajectories) once the robot is close to the target; base motion is also gated by laser safety.
- New Prolog predicates for reasoning/decision making in `prolog/fp_rules.pl`:
  - Inference: `observe_instance/3`, `observation_count/3`, `likelihood_of_surface/3`, `recent_surface/2`, `surface_conflict/1`.
  - Decision: `preferred_surface/2`, `search_target/3`, `needs_confirmation/1`, `decision_to_revisit/2`.

## Useful service checks
```
rosservice call /get_scene_object_list "object_name: 'CupRed'"
rosservice call /report_learning_metrics
rosservice call /compute_target_twist "target_name: 'bookshelf'"
```

## Notes
- The Prolog backend and saved queries are kept under `queries/savedQueries.txt`; launch files default `queries_root` to the package path so the file is picked up automatically.
- The vision hook subscribes to `head_front_camera/rgb/image_raw` by default; override via `vision_topic` in `final_project.launch`.
- `learning_node` asserts knowledge only once per class, then streams observations via `observe_instance/3` for online reasoning.
- `use_direct_perception` and `percept_use_ground_truth` are off by default to avoid Gazebo ground truth; enable them only for debugging.

## Headless verification (FP.T01–T03)
- Build & source (each terminal):
  ```
  cd /home/user/exchange/ssy236_arsule/src/autonomous-reasoning-robot
  source /knowrob_ws/devel/setup.bash
  catkin_make
  source devel/setup.bash
  ```
- Launch Gazebo without GUI:
  ```
  roslaunch world_percept_assig4 gazebo_ssy236.launch gzclient:=false
  ```
- Launch full stack (perception + learning + reasoning + control):
  ```
  roslaunch world_percept_assig4 final_project.launch \
    queries_root:=$(rospack find world_percept_assig4) \
    target_name:=bookshelf \
    use_direct_perception:=false \
    percept_use_ground_truth:=false
  ```
- Optional warm start:
  ```
  rosservice call /load_knowledge "start: 1"
  ```
- Teleop (no UI) to stimulate sensor-based perception:
  ```
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mobile_base_controller/cmd_vel
  ```
- Control & safety signals:
  ```
  rostopic echo -n 5 /mobile_base_controller/cmd_vel
  rostopic echo -n 5 /learning/obstacle_distance
  ```
- Navigation toward a target:
  ```
  rosservice call /compute_target_twist "target_name: 'bookshelf'"
  ```
- Pointing skill messages (after close to target):
  ```
  rostopic echo -n 1 /arm_controller/command
  rostopic echo -n 1 /head_controller/command
  ```
- Learning metrics (online tool hook):
  ```
  rostopic echo -n 1 /learning/confusion_json
  rosservice call /report_learning_metrics
  ```
- Learned/predicted poses:
  ```
  rosservice call /get_scene_object_list "object_name: 'CupRed'"
  rosservice call /get_scene_object_list "object_name: 'all'"
  ```
- Prolog reasoning predicates (FP.T01) via rosprolog shell:
  ```
  rosrun rosprolog rosprolog
  ensure_loaded('package://world_percept_assig4/prolog/fp_rules.pl').
  observe_instance('Bottle','table',1.0).
  observation_count('Bottle','table',C).
  likelihood_of_surface('Bottle','table',P).
  preferred_surface('Bottle',S).
  search_target('Bottle',S,Conf).
  needs_confirmation('Bottle').
  decision_to_revisit('Bottle',S).
  ```










- In two terminals (container, no UI):

  cd /home/user/exchange/ssy236_arsule
  source /knowrob_ws/devel/setup.bash
  catkin_make
  source devel/setup.bash

Launch stack (headless)

- Gazebo without GUI:
  roslaunch world_percept_assig4 gazebo_ssy236.launch gzclient:=false
- Final stack (perception+learning+reasoning+control, sensor-based):

  roslaunch world_percept_assig4 final_project.launch \
  queries_root:=$(rospack find world_percept_assig4) \
  target_name:=bookshelf \
  use_direct_perception:=false \
  percept_use_ground_truth:=false
- Optional warm start: rosservice call /load_knowledge "start: 1"

Drive the robot (no GUI)

- Teleop from another terminal:
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/mobile_base_controller/cmd_vel
- Watch control/obstacle signals:
  rostopic echo -n 5 /mobile_base_controller/cmd_vel
  rostopic echo -n 5 /learning/obstacle_distance

FP.T03 (perception–action + skill)

- Compute/publish a navigation command toward a target:
  rosservice call /compute_target_twist "target_name: 'bookshelf'"
- After you teleop near the target, the controller will also send arm/head pointing commands; confirm traffic:
  rostopic echo -n 1 /arm_controller/command
  rostopic echo -n 1 /head_controller/command

FP.T02 (learning + online tool + evaluation)

- Streamed metrics (online tool hook):
  rostopic echo -n 1 /learning/confusion_json
- Explicit evaluation service (confusion matrix as JSON):
  rosservice call /report_learning_metrics
- Check learned/predicted poses for objects (seen or inferred):
  rosservice call /get_scene_object_list "object_name: 'CupRed'"
  rosservice call /get_scene_object_list "object_name: 'all'"

FP.T01 (reasoning predicates & assertions)

- Open a Prolog shell against the running rosprolog:
  rosrun rosprolog rosprolog
- In the shell, load the rules (if not auto-loaded) and query the new predicates:

  ensure_loaded('package://world_percept_assig4/prolog/fp_rules.pl').
  observe_instance('Bottle','table',1.0).
  observation_count('Bottle','table',C).
  likelihood_of_surface('Bottle','table',P).
  preferred_surface('Bottle',S).
  search_target('Bottle',S,Conf).
  needs_confirmation('Bottle').
  decision_to_revisit('Bottle',S).
  You can also assert a class/instance via the ROS service:
  rosservice call /assert_knowledge "object_name: 'CupRed'"

Run these in sequence: launch Gazebo headless, start final_project.launch, teleop the robot to let sensors observe objects, then issue the T03/T02/T01 checks above. This covers
movement/skill, learning + evaluation, and the new reasoning predicates without needing the Gazebo UI.


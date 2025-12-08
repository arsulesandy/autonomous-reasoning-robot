# SSY236 – Final Project
Group 29  
Sandeep Arsule, Harshith Pidur Kuppusamy

This package implements the final project requirements (perception, reasoning, control and learning). The old `map_generator_node` has been removed; a new `learning_node` now connects perception, reasoning and control, performs online learning, and exposes evaluation metrics.

## Run the full stack
```
cd /home/user/exchange/ssy236_arsule
source /knowrob_ws/devel/setup.bash
catkin_make
source devel/setup.bash

# 1) Simulator
roslaunch world_percept_assig4 gazebo_ssy236.launch

# 2) Perception + learning + reasoning + control (new stack)
roslaunch world_percept_assig4 final_project.launch \
  queries_root:=$(rospack find world_percept_assig4) \
  target_name:=bookshelf

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
rosservice call /direct_pose_srv "object_name: 'cafe_table'"
rosservice call /get_scene_object_list "object_name: 'CupRed'"
rosservice call /report_learning_metrics
rosservice call /compute_target_twist "target_name: 'bookshelf'"
```

## Notes
- The Prolog backend and saved queries are kept under `queries/savedQueries.txt`; launch files default `queries_root` to the package path so the file is picked up automatically.
- The vision hook subscribes to `head_front_camera/rgb/image_raw` by default; override via `vision_topic` in `final_project.launch`.
- `learning_node` asserts knowledge only once per class, then streams observations via `observe_instance/3` for online reasoning. 

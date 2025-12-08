# Assignment 04 – How We Run It
Group 29 Members:
- Student 1: Sandeep Arsule
- Student 2: Harshith Pidur Kuppusamy

Here are some steps which we used during our verification

## Our Basic commands for each terminal while Harshith did this on his own machine
- Workspace we used: `/home/user/exchange/ssy236_arsule`
  ```
    docker exec -it -w /home/user/exchange/ssy236_arsule tiago bash
    source /knowrob_ws/devel/setup.bash
    catkin_make
    source devel/setup.bash
  ```

## Launch stack (Gazebo + perception + reasoning)
Open separate terminals and run in this order:

### 1) Gazebo world
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# roslaunch world_percept_assig4 gazebo_ssy236.launch

Ourput -->
You can start planning now!
[INFO] [1765207197.912634, 3162.863000]: ...connected.
[INFO] [1765207198.636381, 3163.248000]: Gripper opened.
```

### 2) Direct perception
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosrun world_percept_assig4 direct_percept_node

Ourput -->

[ INFO] [1765207414.828348170, 3276.531000000]: Publishing TF for model: cafe_table as frame: cafe_table_direct
[ INFO] [1765207415.690700545, 3276.991000000]: Broadcasted 9 TFs from world.
[ INFO] [1765207417.578609046, 3277.991000000]: Broadcasted 9 TFs from world.
[ INFO] [1765207418.568655671, 3278.531000000]: Publishing TF for model: bookshelf as frame: bookshelf_direct
```

### 3) Map generator
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosrun world_percept_assig4 map_generator_node

Ourput -->

[ WARN] [1765207483.564449465]: Created world map
[ INFO] [1765207509.946726339, 3327.008000000]: Target object: all
[ INFO] [1765207586.349547472, 3367.735000000]: Got new object: bookshelf
[ INFO] [1765207586.351021347, 3367.736000000]: Object Pose: position:
```

### 4) Perception client
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# rosrun world_percept_assig4 percept_node

Ourput -->

[ WARN] [1765209704.290129007]: Created world info
[ INFO] [1765209704.293052466]: Waiting for service update_object_list to be advertised...
[ INFO] [1765209704.294994216]: Connected to service: update_object_list

[ INFO] [1765209689.551094125, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551111334, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551129709, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551152625, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551202875, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551227459, 4493.192000000]: [2]: bookshelf
[ INFO] [1765209689.551247625, 4493.192000000]: [2]: bookshelf
```

### 5) Reasoning
```
root@0d5ad4754c10:/home/user/exchange/ssy236_arsule# roslaunch world_percept_assig4 reasoning.launch queries_root:=/tmp

Ourput -->

[ INFO] [1765208522.402510710, 3866.725000000]: rosprolog service is running.
[ INFO] [1765208522.413630752, 3866.731000000]: waitForService: Service [/rosprolog/query] is now available.
[ INFO] [1765208522.418255127, 3866.734000000]: query_file: /tmp/queries/savedQueries.txt
[ INFO] [1765208522.418463668, 3866.734000000]: query_file: /tmp/queries/savedQueries.txt
[ INFO] [1765208568.559161843, 3891.302000000]: Got new object: bookshelf
[ INFO] [1765208568.559254926, 3891.302000000]: query: get_class('bookshelf').
New class created: bookshelf
[ INFO] [1765208568.570861801, 3891.309000000]: A new class was created in the ontology
[ INFO] [1765208568.572384885, 3891.309000000]: query: create_instance_from_class('bookshelf',0,Instance).
[ WARN] [1765208568.575264260, 3891.311000000]: new instance in knowledge base: http://www.chalmers.se/ontologies/ssy236Ontology.owl#bookshelf_0
```

## Knowledge loader
```
rosparam load $(rospack find world_percept_assig4)/config/loadKnowledge.yaml
```

```
rosrun world_percept_assig4 knowledge_node /tmp

[ INFO] [1765208700.500171626]: Wait for the Prolog service...
[ INFO] [1765208700.517417876]: Loading queries from: /tmp/queries/savedQueries.txt
```

```
rosservice call /load_knowledge "start: 1"

confirm: True
```

## Controller check (A04.T04)
```
rosservice call /compute_target_twist "target_name: 'bookshelf'"

cmd_vel:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: -1.6671051823423127e-05
success: True
message: "Command computed"
```

## Basic service checks
```
rosservice call /direct_pose_srv "object_name: 'cafe_table'"
```

```
rosservice call /get_scene_object_list "object_name: 'all'"
```

```
rosservice call /assert_knowledge "object_name: 'CupRed'"
```

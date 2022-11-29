# messages to compute IK solutions #



## Description
This package provide messages to compute all the IK solutions for a TF

### msg

- __Configuration__

```yaml
float64[] configuration #joint state
```

- __IkSolution__

```yaml
ik_solver_msgs/Configuration[] configurations
```

- __CollisionResult__

```yaml
ik_solver_msgs/Configuration solution
string[] colliding_obj_first
string[] colliding_obj_second
bool out_of_bound
float64 feasibility # 0 collision, 1 fully feasible
float64 distance    # distance w.r.t. obstacles
time stamp
````

### srv

- __GetIk__

```yaml
string tf_name
ik_solver_msgs/Configuration[] seeds
string[] seed_joint_names
uint32 max_number_of_solutions # if zero, use default.
uint32 stall_iterations # if zero, use default.
---
ik_solver_msgs/IkSolution solution
string[] joint_names
```

- _tf_name_ name of the TF where the IK has to be computed

- _seeds_ list of configurations used as seeds (not mandatory)

- _solution_ list of solutions

- _joint_names_ list of joint names

- __GetIkArray__

```yaml
geometry_msgs/PoseArray poses

ik_solver_msgs/Configuration[] seeds
string[] seed_joint_names
uint32 max_number_of_solutions # if zero, use default.
uint32 stall_iterations # if zero, use default.
---
ik_solver_msgs/IkSolution[] solutions
string[] joint_names
```

- _poses_ list of poses (frame defined in header/frame_id)

- _seeds_ list of configurations used as seeds for the first pose (not mandatory). Solutions of previous pose are used as seeds for the following one.

- _solutions_ list of list of solutions

- _joint_names_ list of joint names


- __NeighbourhoodPyramidIk__

Check inverse kinematics in a Neighbourhood Pyramid around a tf names _tf_name_

```yaml
# this service compute the IK for a pyramid with apex in [tf_name] frame and a square base.
# the four lateral faces are tested, while the inner volume is tested along the diagonals.

string tf_name # reference frame of the pyramid
uint32 max_number_of_solutions # if zero, use default.
uint32 stall_iterations # if zero, use default.

# the neighbourhood pyramid has a square base in the plane XY
float64 width # width of the pyramid base
float64 distance # height of the pyramid (in +Z direction)
float64 resolution # distance between testing points

# for each point of the pyramid the following orientation (referred to tf_name frame)
# are tested:
# rpy: [0,0,0]
# rpy: [+roll,+pitch,+yaw]
# rpy: [+roll,+pitch,-yaw]
# rpy: [+roll,-pitch,+yaw]
# rpy: [+roll,-pitch,-yaw]
# rpy: [-roll,+pitch,+yaw]
# rpy: [-roll,+pitch,-yaw]
# rpy: [-roll,-pitch,+yaw]
# rpy: [-roll,-pitch,-yaw]
float64 roll
float64 pitch
float64 yaw
---
# number of poses with cannot be reached by any IK solution
int32 number_of_unreachable_poses

# list of tested poses
geometry_msgs/PoseArray poses

# list of list of ik solutions
ik_solver_msgs/IkSolution[] solution
string[] joint_names
```

- __CollisionChecking__

```yaml
ik_solver_msgs/Configuration[] solutions
string[] joint_names
bool detailed # compute distances and colliding links
moveit_msgs/PlanningScene planning_scene
---
ik_solver_msgs/CollisionResult[] result
```

## Test
run the node _get_tf_ik.py_ providing the name of the IK server and the name of the desired tf. Example:
```
rosrun rosdyn_ik_solver get_tf_ik.py [server_name] [tf_name]
```
The node publish a _moveit_msgs/DisplayRobotState_ topic called _ik_solution_. The robot state shows cyclically the IK solution.

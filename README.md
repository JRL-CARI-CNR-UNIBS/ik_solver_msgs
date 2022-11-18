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

- __CollisionChecking__
```yaml
ik_solver_msgs/Configuration[] solutions
string[] joint_names
---
ik_solver_msgs/CollisionResult[]
```

## Test
run the node _get_tf_ik.py_ providing the name of the IK server and the name of the desired tf. Example:
```
rosrun rosdyn_ik_solver get_tf_ik.py [server_name] [tf_name]
```
The node publish a _moveit_msgs/DisplayRobotState_ topic called _ik_solution_. The robot state shows cyclically the IK solution.

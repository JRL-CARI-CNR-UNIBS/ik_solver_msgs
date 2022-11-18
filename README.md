# messages to compute IK solutions #



## Description
This package provide messages to compute all the IK solutions for a TF

### msg

- _Configuration_ joint state

```yaml
float64[] configuration #joint state
```

- __CollisionResult__

```yaml
ik_solver_msgs/Configuration solution
float64 feasibility # 0 collision, 1 fully feasible
string[] colliding_links # compute only is detailed collisions are available
float64 distance    # distance w.r.t. obstacles. compute only is detailed collisions are available
````

### srv

- __GetIk__

```yaml
string tf_name
ik_solver_msgs/Configuration[] seeds
---
ik_solver_msgs/Configuration[] solutions
string[] joint_names
```

- _tf_name_ name of the TF where the IK has to be computed

- _seeds_ list of configurations used as seeds (not mandatory)

- _solutions_ list of solutions

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

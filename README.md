# messages to compute IK solutions #



## Description
This package provide messages to compute all the IK solutions for a TF

### msg

- Configuration: joint state
```yaml
float64[] configuration
```

### srv

```yaml
string tf_name
ik_solver_msgs/Configuration[] seeds
---
ik_solver_msgs/Configuration[] solutions
```

- _tf_name_ name of the TF where the IK has to be computed
- _seeds_ list of configurations used as seeds (not mandatory)
- _solutions_ list of solutions

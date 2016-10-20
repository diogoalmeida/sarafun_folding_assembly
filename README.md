Folding assembly controller
=============

This package provides the folding assembly functionality to the SARAFun project. It implements two actionlib servers:

- The complete folding assembly controller
- A standalone implementation of the contact point estimator

Both implementations offer the ability to configure how the algorithms are executed. This file details how to use the implemented servers.

Triggering an actionlib server manually
---
Actionlib provides the user with a simple graphical interface that allows for sending goal messages and monitor feedback, as well as preempting the action. Preemption and goal sending is handled by buttons in the GUI. On success, preemption or abortion, the result is displayed. To run the action client you need to type, in a terminal shell:

```$ rosrun actionlib axclient.py /action_name action_message```

![Actionlib GUI](http://imgur.com/a/6bM00)

Estimator action server
---
The contact point estimator uses force and torque measurements to compute the position of the contact point with respect to the sensor frame.
The actionlib interface uses an empty goal to trigger the estimator, and provides feedback on the current estimate. You can call the estimator using the action client.

```$ rosrun actionlib axclient.py estimator_node folding_assembly_controller/EstimateAction```

The estimator is configured on the ```config_estimator.yaml``` file, located in the ```input/``` directory of the package. There, the user can adjust the kalman filter process and measurement noise matrices, define the initial covariance estimate and set the force torque ROS topic name. The actionlib server updates the estimate at the frequency set in this file.

Folding assembly action server
---
The folding controller is implemented with an actionlib interface that allows for testing its components individually. The user can choose whether to apply force control, the direction of the force compensation and whether to use contact point estimation, with options for setting a hardcoded contact point, use direct force and torque measurements or apply a kalman filter estimate. All these options are explained in more detail in the action definition, in the ```action/``` directory. You can call the controller using the action client.

```$  rosrun actionlib axclient.py folding_assembly_node folding_assembly_controller/FoldingAssemblyAction```

The controller is configured on the ```config_controller.yaml``` file, where the user can the end-effector linear and angular velocity saturation values, controller gains, hardcoded contact point distance and relevant frame names.  

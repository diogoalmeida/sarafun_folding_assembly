Folding assembly controller [![Build Status](https://travis-ci.org/diogoalmeida/sarafun_folding_assembly.svg?branch=master)](https://travis-ci.org/diogoalmeida/sarafun_folding_assembly)
=============

This package provides the folding assembly functionality to the SARAFun project.
It controls the relative pose between two rigid objects in contact, along two
motion directions: a translational direction and a rotational axis, which are
non-collinear. It also provides adaptive estimators that will estimate kinematic
uncertainties on the task, such as the available motion directions and contact
point location. Besides a dual-armed robot integrated with ROS, the package assumes
the existance of at least one force and torque sensor.

![ECTS](https://lh5.googleusercontent.com/XdnL-8iklN4WCx3oGqZWsEaHeJjMkAc8Nxqyv4fH-COVXFjeuOdLWMgighJXceyUth-0TfE5nV5O-W1TkDsg=w1920-h1037-rw)

Dependencies
===

* [sarafun_msgs (link to dummy package)](https://github.com/diogoalmeida/sarafun_msgs)
* [generic_control_toolbox](https://github.com/diogoalmeida/generic_control_toolbox)
* abb_irb14000_support
* [ROS Indigo](http://www.ros.org/)

Running the controller
===
The file 'config_action_node.yaml' must be edited in order to set the reference frames
used for the controller this incudes the name of the KDL end-effector frame and
the gripping frame name in which the controller quantities are expressed.
The controller is launched with

``
$ roslaunch folding_assembly_controller controller.launch
``

After launch, and actionlib server will be running which can be called as

``
$ rosrun actionlib axclient.py /folding_controller/<action_name>
``

where `<action_name>` is set in the configuration file. You should see the following GUI:

![actionlib](https://lh5.googleusercontent.com/I1fmjDadn1zQhW7X1KqIdp3Fm5K5KpAseA5T1lp9GAmB3zmdhuCt3hGgp8WXFw6jR8VVho762TyEXwQWoV-O=w1920-h1037-rw)

The `init_t_error` and `init_k_error` variables are useful for controller experiments.
`goal_force` sets the desired contact force along the estimated surface normal.

The controller operates on two different modes: pose control or velocity reference.
In pose control (`use_pose_goal` set to true), the robot will regulate the two rigid parts to a relative pose along the two motion directions. Alternatively (`use_pose_goal` set to false), the system will command velocities along the estimated motion directions.

Dynamic Reconfigure
===
A dynamic reconfigure server is provided for facilitating testing the system.
Notably, it allows setting the degree to which the arms share the manipulation task, through the alpha parameter under the `ects` group.

![reconfig](https://lh6.googleusercontent.com/joEmZUHQ3c-m6LtXO3oU-Z3JD0jIYKJgn_HM5dpCmS8twb1HW5er00Z9JbfE7Iff1WPSJI1voIewkcISMYhx=w1920-h1037-rw)

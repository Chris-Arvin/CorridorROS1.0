# mpc_tracer-for-ROS
mpc_tracker for ROS, wrapped as a local_planner plugin.

* A package "api2python" follows the format of local_planner in ROS navigation stack. It calls the mpc tracer by SERVICE. 
* Another package wraps Model Predictive Control (MPC) by python and cvxpy. It returns the action for the robot. 

A reference of MPC can be found [here](https://blog.csdn.net/weixin_43879302/article/details/105880972).
The packages are highly recommanded to be equiped with our simlation platform [HallwayROS1.0](https://github.com/Chris-Arvin/HallwayROS1.0).

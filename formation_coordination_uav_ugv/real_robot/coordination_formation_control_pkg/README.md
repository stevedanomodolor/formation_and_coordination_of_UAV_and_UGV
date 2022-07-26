https://askubuntu.com/questions/1372425/gpg-keyserver-receive-failed

install raspberry
https://howchoo.com/pi/raspbian-buster-install-or-upgrade

interact with ssh
http://wiki.ros.org/ROS/Tutorials/MultipleMachines#rostopic

TODO:
to execute this package you need the crazyswarm library, configure it as you please, there is a detailed explanation on how to do so
This code uses a differenc onfiguration but this is independent and should not affect the execution of this package

TODO
implement modules
- [X] send command, take off, land
- [X] send command should include conversion to velocity
- [X] test omniwheels node
- [X] Implement swam controller
- [X] filter received normal velocity and use it for the integrator
- Redo the following experiments
  - orientation, make sure it last longer
  - flocking- las longer with shorter velocity 0.001



How to execute the plot result
python plot_results.py ../results/experiment_1_trianglular_formation_crazyflie_1_logdata.bag crazyflie_1 3

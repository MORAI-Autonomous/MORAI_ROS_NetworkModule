[![MORAILog](./docs/MORAI_Logo.png)](https://www.morai.ai)


 

 

# Installation

Install packages which basically need

```
$ git clone https://github.com/MORAI-Autonomous/MORAI_ROS_NetworkModule.git
$ cd MORAI_ROS_NetworkModule
$ git submodule update --init --recursive
$ sudo chmod -R a+x catkin_ws/src/
$ cd catkin_ws/src && catkin_init_workspace 
$ cd ..
$ catkin_make
$ source devel/setup.bash
```


teo_batc
=========

ROS packages for TEO humanoid robot with Moveit! and the batc method.

##Install
This package is devoloped for the ROS INDIGO version, any different ROS version may not be supported. For the correct setup of the packages follow the next steps.

1. **Clone** this repository in a catkin_workspace. See this totorial <http://wiki.ros.org/catkin/Tutorials/create_a_workspace> to create a catkin_workspace in ROS.

2. Use **rosdep** to install all the needed dependencies. You also need to have installed the **teo-ros-simulation** (<https://github.com/roboticslab-uc3m/teo-ros-simulation>) repository, in your computer.

	```
		rosdep install teo_moveit

		rosdep install teo_batc
	```

3. **Compile** the project with the following commands

	```
		cd "your_catkin_ws_route"

		catkin_make	
	```


##Teo_moveit package
This package contains several examples of the TEO humanoid robot using the state of the art software Moveit!. 

**Note: Teo_moveit_config** is the generated package using the moveit! assistant, for the correct TEO configuration in Moveit!. Additional info about this assistant can be found in <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>.

For further documentation about this package take a look at the readme.md in its repository.
##teo_batc
This package contains the implementation of a Base Trajectory Combination (**BATC**) system. A novel method for the **combination of different trajectories in the robot memory, to generate of a new one**. Developed in the project "new task generation for humanoid robots based on case retrieval and user communication". 

A more detailed explanation about the package is given in the readme.md in the same package.

##teo_description
Contains the urdf files for the TEO description in the ROS environment. This packege was extracted from <https://github.com/roboticslab-uc3m/teo_robot>.
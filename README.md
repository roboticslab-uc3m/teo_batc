teo_batc
=========

ROS packages for TEO humanoid robot with Moveit! and the batc method.

- The master branch is updated with the latest version of packages.
- The development branch contains the main development of new versions.

##Teo_moveit package
This package contains several examples of the TEO humanoid robot using the state of the art software Moveit!. 

**Note: Teo_moveit_config** is the generated package using the moveit! assistant, for the correct TEO configuration in Moveit!. Additional info about this assistant can be found in <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>.

For further documentation about this package take a look at the readme.md in its repository.
##teo_batc
This package contains the implementation of a Base Trajectory Combination (**BATC**) system. A novel method for the **combination of different trajectories in the robot memory, to generate of a new one**. Developed in the project "new task generation for humanoid robots based on case retrieval and user communication". 

A more detailed explanation about the package is given in the readme.md in the same package.

##teo_description
Contains the urdf files for the TEO description in the ROS environment. This packege was extracted from <https://github.com/roboticslab-uc3m/teo_robot>
#teo_moveit
This package contains several examples of the TEO humanoid robot using the state of the art software Moveit!. 

**Note: Teo_moveit_config** is the generated package using the moveit! assistant, for the correct TEO configuration in Moveit!. Additional info about this assistant can be found in <http://docs.ros.org/hydro/api/moveit_setup_assistant/html/doc/tutorial.html>.

##Moveit! manipulation examples
###Basic world
To run a **basic scenario** using Moveit! and TEO, use the following command.

```
roslaunch Teo_moveit_config demo.launch
```

**Important note:** In this first version the reset of the system when calling a new one is not implemented, so for changing between the different worlds, the previous reset of the Moveit! environment, is recommended. If this is not done the worlds will overlap.
###Water a plant task example (Garden world)
Two basic tasks examples using Moveit! and TEO are also provided. For running the example **water a plant** in the garden world, enter the following command in a terminal:

First init the teo and moveit system.

```
roslaunch teo_moveit teo_cbr_startup.launch
```
Once the system is initialized run the following command in a new terminal.

```
rosrun teo_moveit water_plant_task 
```
###Drink coffee task example (Kitchen world)
For running the example **drink a coffee** in the kitchen world, enter the following command in a terminal:

First init the teo and moveit system.

```
roslaunch teo_moveit teo_cbr_startup.launch
```
Once the system is initialized run the following command in a new terminal.

```
rosrun teo_moveit drink_coffee_task
```
###CBR system

In addition of this examples this package is intended to be a base for the future development of a CBR system for robotics. This way a planification and communication step are implemented using the basic structure of a CBR system. **This system allow us to communicate new tasks to the system, and later be executed in Moveit!.**

In order to use this system run the following command in a terminal:

```
roslaunch teo_moveit teo_cbr_startup.launch
```

Once the system is initialized run the following command in a new terminal.

```
rosrun teo_moveit central_node
```

The structure presented in this package is the following:

![alt tag](http://i.imgur.com/U30MsFR.jpg?1)

**data_communication** is the array that contains the information obtained from the user in the communication step and that is latter transmited between nodes

- **High_task**: name of the performing task, no relevant for planification.

- **Object**: name of the object we want to manipulate, no relevant for planification, since in this first version, we only have one manipulable object, in each environment.

- **atomic_task**: The low_level tasks we want to perform (The user is asked for the low level structure of the task). The implemented low-level tasks are move arm, pick object and place object. More atomic task can be added in the code.

- **move_group**: The name of the *move_group* we want to move in Moveit! in this case we have *right_arm* or *left_arm*.

- **Positions**: The predefined positions where we want to move the end-effector. In this first version there are only two predefined position correspondin to the Kitchen scenario. They are *close_cup* (Move the end-effector to a position close to the cup), and *close_mouth* (Move the end-effector to a position close to the TEO "mouth"). More predefined positions can be easly added in the code. There is also the option to direclty send the coordinates to the robot, although this is not implemented in the communication step, since is not part of a normal user-robot communication.

**data_struct** Is the low_level data generated from the high level instruction from the user in order to be executed in Moveit!ed in Moveit!

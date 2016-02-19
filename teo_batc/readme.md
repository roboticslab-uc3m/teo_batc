#teo_batc

This package contains the implementation of a Base Trajectory Combination (**BATC**) system. A novel method for the **combination of different trajectories in the robot memory, to generate of a new one**. Developed in the project "new task generation for humanoid robots based on case retrieval and user communication". 

To use this system enter the following commands:

First init the Moveit! environment:

```
roslaunch teo_moveit teo_cbr_startup.launch
```
Init the BATC system

```
roslaunch teo_batc teo_trajectory.launch
```

Initially this system only works with two base trajectories. *Note*: New trajectories can be added uncommenting the  additional trajectories in the code of *trajectory_memory.cpp*.

How this package works can be seen in the following schema.

![alt tag](http://i.imgur.com/c7l99Uy.png?1)

- **Trajectory memory** This node, is the one in charge to act, like the database
of the system. Here all the tasks are stored, with the characteristics that
defines them, and that will be used for the indexing problem. This means using different
arrays of two dimensions to store things, such as, the characteristics of the
trajectory, or the splines values that defines it.

- **Trajectory base solution** This subsystem is used to generate the base solution, using the trajectories stored in ”trajectory memory”. This is done, the same way, as the proposed in chapter two. First, the two best trajectories of the database, are selected, and then the weights assigned to it, are calculated, using for that the final trajectory information. To finish these weights, are returned, to the caller function.

- **Main trajectory** This is the central node in the trajectory block, it is in
charge of the startup, control, and communication, of the two other parts
of the system. Here is where the user defines the final desired trajectory,
that will be send to the ”trajectory base solution”, and where the weights,
returned by this node, are used to perform the weighted sum. Finally, this
information will be send to the ”central node”, to be executed in the sim-
ulation.

- **Central node** This is the central node of the BATC system. It is in charge
of the initialization. This is the node that has to be
executed, to start the simulation, since, is the one in charge to call the other
nodes in the correct sequence, and make sure that everything is working
well.o call the other
nodes in the correct sequence, and make sure that everything is working
well.


# THE 2D MATRIX

![a floating shape ros Demo](https://raw.githubusercontent.com/1hada/spawn_urdf_cube_matrix/master/cube_matrix_100x100.gif)


With these files you will be able to make a floating shapes with ROS in Gazebo .
I made 3 relevant files, one that is able to construct 2d matrices, another 3d,  and another which 
produces a gui with buttons that allows one to rotate the matrices with 6 degrees of freedom.
The rotation is made possible through its pose ( roll, pitch, and yaw).

The ./a_matrix executable file is set to make a 2D matrix of cubes ( made from matrix_spawn_2d.cpp ).
ATTENTION : 
            The matrix has a 100x100 dimension, this may take a significant amount of minutes
            to spawn ( or may not spawn at all depending on your computer).

            Of course if you don't want to have to wait to spawn all those cubes then, 
            go into the cpp file and change the dimension of the matrix.

Regardless of what you do with the dimension of the matrix this README will show
how to spawn in gazebo with c++ . If you want to spawn a different object in gazebo using c++ 
then I sincerely hope these files will help guide you to be able to do that. Enjoy !


You wont find any files in the URDF directory because all gazebo shapes are spawned in the c++ file.

# THE 3D MATRIX

![a floating shape ros Demo](https://raw.githubusercontent.com/1hada/spawn_urdf_cube_matrix/master/cube_matrix_8x8x8.gif)




# Prerequisites

I will be assuming that you have :

	Ubuntu 18.04 
	ROS melodic
	gazebo9

These are the steps I took to spawn the matrix,
they may be different depending on how you
set-up / use ros on your computer.

# STEP1
Go to you catkin workspace "src" directory.

Make the package.

```
cd ~/catkin_ws/src
catkin_create_pkg spawn_urdf_cube_matrix
```
You can move the files you've cloned from this repository into that directory later if it is easier for you.

# STEP2
Make sure the package is known to ROS.
```
cd ~/catkin_ws
. ~/catkin_ws/devel/setup.bash
```
# STEP3
Launch the world where the matrix will exist. 
NOTE : I have not included the 
```
roslaunch spawn_urdf_cube_matrix a_matrix_world.launch
```

# STEP4 
I've attached the command structure you'd need if you want to recompile the c++ code with the ROS headers.

```
cd ~/catkin_ws/src/spawn_urdf_cube_matrix/src
```
2D matrix
```
g++ matrix_spawn_2d.cpp -o a_matrix -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization

./a_matrix
```
3D matrix
```
g++ matrix_spawn_3d.cpp -o a_matrix -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization

./a_matrix
```
Controls
```
g++ -Wall -Wno-format move_cube_gui.cpp -o execute_move_cube -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization -Wno-deprecated-declarations -Wno-format-security -lm `pkg-config --cflags --libs gtk+-3.0` -export-dynamic

./execute_move_cube
```



P.S. 
If you turn the sun off the matrix will spawn into gazebo a tad bit faster. I left it on in the world file because I like the way the cubes look with light.




### Common Error Messages You'll Encounter When Forking and Writing Your Own 

```
urdf.cc:3170] Unable to call parseURDF on robot model
Error [parser.cc:406] parse as old deprecated model file failed.
```
Means you might have:
	-Made typo/ links with the same name.
	-Left out a necessary field.
	-Used the wrong format. ( maybe you used sdf instead of urdf format )
see  : http://gazebosim.org/tutorials/?tut=ros_urdf   for exact needs
I am also including my own list of SDF <---> URDF conversions that one should be aware of. 


The following will happen if you get to excited and try making enough cubes to piss the source code off .

```
[gazebo-2] process has died [pid 3148, exit code 137, cmd /opt/ros/melodic/lib/gazebo_ros/gzserver -e ode /home/USERNAME/catkin_ws/src/spawn_urdf_cube_matrix/worlds/a_box.world __name:=gazebo __log:=/home/USERNAME/.ros/log/6311b964-4087-11ea-8c1b-24fd52f98ca1/gazebo-2.log].
log file: /home/USERNAME/.ros/log/6311b964-4087-11ea-8c1b-24fd52f98ca1/gazebo-2*.log
```
And 
```
[ERROR] [1579987268.006795207, 50.489000000]: a message of over a gigabyte was predicted in tcpros. that seems highly unlikely, so I'll assume protocol synchronization is lost.
``` 


### SIDE NOTE

I am aware that the links are unified into one. As an encouraged exercise I leave it to the reader to 
play with the code and make it so that the links are shown as individuals.
I post this gif here to show that it is possible to do so. 
Have Fun !

![link presence ros Demo](https://raw.githubusercontent.com/1hada/spawn_urdf_cube_matrix/master/link_individuality_note.gif)








Since you made it this far.
Hint : Consider the joints.







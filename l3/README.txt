#CS4610: Robotic Science and Systems
------------------------------------

##Overview
----------
This lab involved programming the robot to pick up a "flag" using its arm.  

##Compiling
----------
run make

##Running
---------
for pickup code:
	./run-class Arm x-coordinate-of-flag y-coordinate-of-flag
for key control:
	./run-class Control

##Problems
----------
When gripper length longer than 260 mm, after pressing “a” or “d” you 
can see the gripper extend itself obviously to maintain the y coordination in 
the previous robot frame. Otherwise the gripper barely move after passing 
“a” or “d�.

The code for deciding if a point is reachable is a bit buggy.  It can generally
prevent the arm from reaching bad points, but it has a lot of false negatives.


##Who Did What?
---------------
Nick was absent for the first lab for an interview.  As such, Qimeng and Zach 
started the project.  Zach started by writing the inverse kinematics code and 
a lot of the starting code in Arm.  Qimeng implemented the forward kinematics 
and the key control code.  Nick implemented the point interpolation and 
cleaned up the code in both files.

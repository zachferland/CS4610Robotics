CS4610: Robotic Science and Systems
Lab 5: Fiducial Tag Localization and EKF
Nicholas Jones, Zach Ferland
-----------------------------------

How to compile
--------------
hopefully the code will run without installing anything new

run: make

If this is not the case you may need to follow the installation instructions
on http://april.eecs.umich.edu/wiki/index.php/Download_and_Installation

How to Execute
--------------
Unfortunately we don't have a cohesive product.

to run the tag localization:
	./run-class FiducialTagFinder

The tag used for testing was tag 0 of tag family tag36h11, which is available
here: http://april.eecs.umich.edu/software/tag36h11.tgz
The size of the tag will effect the accuracy of any 3d readings (were we making
them correctly.)  For quick demonstration purposes this shouldn't matter.

to run the EKF:
	./run-class EKF

EKF can also be run in another class in a seperate thread as such
       EKF ekf = new EKF(ohmm);
       ekf.run();

Known Bugs/ Incomplete aspects
------------------------------
AprilTag Localization:
       As far as I can tell, the only pieces that prevent the localization from
       working perfectly are the two coordinate tranformations.  In 
       FiducialTagFinder.getVisibleTagLocs(), the homography needs to be converted
	into the location of the tag in reference to the robot.  We were unable to
       get this to work.  In WorldMap.findRobotGlobal(), the location of the tag
       in the robot frame needs to be compared to its location in the world frame
       to find the pose of the robot.  We were unable to get this to work either.

	Other than these 6 lines of code, I'm pretty sure everything works.

EKF: 
EKF runs quite well and accurate. It has to be tuned more properly though with more
accurate error estimates to run well for this specific robot. The current EKF class fakes
input (pose) from an extoreceptive sensor(camera) since we were not able
incorporate AprilTags properly and get accurate readings from it. There is a few minor
things noted in the code that could be improved and written more accurately as well. I would 
like to improve it in the future and provide a visualization of the error ellipse.


Extra Credit
------------
None

Contributions
-------------
Nick wrote all the code in WorldMap, TagLocation, and FiducialTagFinder.

Zach wrote the code in EKF
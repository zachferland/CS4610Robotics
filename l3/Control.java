/*******************************************************************************
 *
 * Robotic Systems and Science: CS4610
 * Lab 3: Arm Kinematics and Grasping
 * Zach Ferland, Nick Jones, Qimeng Song
 *
 *******************************************************************************/

package l3;

import java.io.IOException;

import ohmm.*;

/**
 * Control provides a way of manipulating the Ohmm arm using the keyboard
 * @author Qimeng Song
 * @version 1/12/2014
 *
 * Controls:
 *  w - move the gripper +d mm in the world frame x direction
 *  s - move the gripper -d mm in the world frame x direction
 *  a - move the gripper +d mm in the world frame y direction
 *  d - move the gripper -d mm in the world frame y direction
 *  r - move the gripper +d mm in the world frame z direction
 *  f - move the gripper -d mm in the world frame z direction
 *  q - set d as min(d + 5, 20)
 *  z - set d as max(d - 5, 5)
 *
 * Assume that robot has initial world frame pose of (0, 0, 0)
 */
public class Control {

	/**
	 * Read key input from user modifying gripper location based upon it
	 * @param ohmm - The robot to control
	 */
	private static void readKey(OHMMDrive ohmm) throws IOException,
	InterruptedException{
		float gx = 0;
		float gz = 0;
		float[] gR = currentGripperLoc(ohmm);
		gx = gR[0];  //gripper x coordination to the Robot frame
		gz = gR[1];  //gripper z coordination to the Robot frame
		double alpha = 0;
		//		double theta = 0;
		float d = 5f;
		for (;;) {
			System.out.print("hit any key: ");
			int c;

			c = ConsoleNonblocking.getChar();
			System.out.println("int value "+c+"; ASCII char \'"+((char)c)+"\'");


			switch (c) {
			case ConsoleNonblocking.ESC : 
				System.exit(0);
			case 'w' : 
				gx += d;
				if(Arm.isReachable(gx, 0, gz)){
					Arm.setArm(Arm.invK(gx, gz), ohmm);
				}
				else{
					gx -= d;
					System.out.println("not reachable");
				}
				break;
			case 's' : 
				gx -= d;
				if(Arm.isReachable(gx, 0, gz)){
					Arm.setArm(Arm.invK(gx, gz), ohmm);
				}
				else{
					gx += d;
					System.out.println("not reachable");
				}
				break;
			case 'a' : 
				alpha = Math.atan2(d, gx);
				gx += d * Math.sin(alpha);
				Arm.setArm(Arm.invK(gx, gz), ohmm);
				while (ohmm.armActive()){}; 
				ohmm.driveTurn((float) alpha);
				break; 
			case 'd' : 
				alpha = Math.atan2(d, gx);
				gx += d * Math.sin(alpha);
				Arm.setArm(Arm.invK(gx, gz), ohmm);
				while (ohmm.armActive()){}; 
				ohmm.driveTurn((float) -alpha);
				break; 
			case 'r' :
				gz += d;
				Arm.setArm(Arm.invK(gx, gz), ohmm);
				break;
			case 'f' :
				gz -= d;
				Arm.setArm(Arm.invK(gx, gz), ohmm);
				break;
			case 'q' : 
				d = Math.min(d + 5, 20f);
				System.out.println(d);	
				break;
			case 'z' : 
				d = Math.max(d - 5, 5);
				System.out.println(d);
				break;
			}
			while (ohmm.armActive()){}; 
			System.out.println("gx: "+gx+", gz: "+gz +", alpha: "+alpha);
		}

	}



	/**
	 * Get the current gripper location in the Robot coordinate frame
	 * @param ohmm - The robot to control
	 * @return The x, y coordinates of the gripper tip in the Robot frame
	 */
	public static float[] currentGripperLoc(OHMMDrive ohmm){
		float[] gR = new float[2];
		float wz = 19f;
		float wx = 74f;
		float sz = 91f;
		float sx = 38f;
		float l0 = Arm.len0;
		float l1 = Arm.len1;

		float[] theta = ohmm.armGetAllJointsRad();
		float c0 = (float)Math.cos(theta[0]);
		float s0 = (float)Math.sin(theta[0]);

		float c012 = (float)Math.cos(theta[0]+theta[1]+theta[2]);
		float s012 = (float)Math.sin(theta[0]+theta[1]+theta[2]);

		float c01 = (float)Math.cos(theta[0]+theta[1]);
		float s01 = (float)Math.sin(theta[0]+theta[1]);

		gR[0] = c012*wx +s012*wz+c01*l1+c0*l0+sx;
		gR[1] = -s012*wx+c012*wz-s01*l1-s0*l0+sz;
		return gR;
	}


	/**
	 * ENTRY POINT
	 */
	public static void main(String[] argv) throws IOException, 
	InterruptedException {
		OHMMDrive ohmm =
				(OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});

		if (ohmm == null) System.exit(0);

		ohmm.armEnable(true);
		//ohmm.armSetAllJointsRad(Arm.invK(220f, 45f));
		//while (ohmm.armActive()); // Wait for arm to stop moving
		ohmm.armHome();
		while(ohmm.armActive());

		readKey(ohmm);
	}
}
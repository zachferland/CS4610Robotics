/*******************************************************************************
*
* CS4610: Lab 4
* Group 1: Nicholas Jones, Zach Ferland, Qimeng Song
*
*******************************************************************************/

package l4;

import java.io.IOException;

//import l3.Grasp;
import ohmm.*;

/**
* This class contains the P controller for the motion of the robot.
* 
* The most important method is calcVW which takes the x, y pixel coordinates of
* the ball in the image and calculates the required linear and radial velocity
* required for the robot to put the ball closer to the PICKUP_LOCATION.
*
* @author Nick Jones
* @version 3/25/2014
*/
public class VWController {
    public static final float SPEED_COEFF = .5f;
    public static final float ROTATION_COEFF = .5f;
	private int[] PICKUP_LOCATION;

    /**
    * Constructor
    * @param pickupLoc - The {x, y} pixel coordinates in the camera image that 
    *                    the ball needs to be at to be picked up. 
    *                    Origin in top left
    */
    public VWController(int[] pickupLoc) {
        this.PICKUP_LOCATION = pickupLoc;
    }

    /**
    * if the ball is at (x, y) in the image (graphical coordinates), calculate
    * the linear (v) and radial (w) velocities that will bring the ball closer
    * to the PICKUP_LOCATION
    * @param x - x pixel coordinate of the ball in the image (graphical coord)
    * @param y - y pixel coordinate of the ball in the image (graphical coord)
    * @return {linearVel, angularVel}
    */
    public float[] calcVW(int x, int y) {
        int dx = this.PICKUP_LOCATION[0] - x;
        int dy = this.PICKUP_LOCATION[1] - y;

        System.out.printf("dx: %d, dy: %d\n", dx, dy);
        float v, w;
        //float v = this.SPEED_COEFF * dy;
        //float w = this.ROTATION_COEFF * dx;

        w = dx / 200f * .5f;
        v = dy / 200f * 150f;

        return new float[]{v, w};
    }

    /**
    * Is the robot at correct location to pick up the ball?
    * @param x - x pixel coordinate of the ball in the image (graphical coord)
    * @param y - y pixel coordinate of the ball in the image (graphical coord)
    * @return true iff the ball is within 5 pixels of the correct x and y in the
    *         image.
    */
    public Boolean atPickupLocation(int x, int y) {
        int dx = x - PICKUP_LOCATION[0];
        int dy = y - PICKUP_LOCATION[1];

        return Math.sqrt(dx * dx + dy * dy) < 10;
    }
    
    public static void main(String argv[]) throws IOException {
    	OHMMDrive ohmm =
				(OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
    	if (ohmm == null) System.exit(0);
    	ohmm.armEnable(true);
		ohmm.armHome();
		while(ohmm.armActive());
    	Grasp grasp = new Grasp(ohmm);
    	grasp.ikDemo();
	}	
}
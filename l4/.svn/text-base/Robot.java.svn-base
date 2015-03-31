/*******************************************************************************
*
* CS4610: Lab 4
* Group 1: Nicholas Jones, Zach Ferland, Qimeng Song
* 
*
*******************************************************************************/
package l4;

import java.io.IOException;

import ohmm.*;
import ohmm.Grasp.*;


public class Robot {

    public static void setupOhmm(OHMMDrive ohmm) {
        // Set up robot
        ohmm.armEnable(true);
        ohmm.driveSetPose(0f, 0f, 0f);
        ohmm.armHome();
        while(ohmm.armActive());
    }

    public static void pickup(OHMMDrive ohmm) throws IOException {
        // run arm pickup sequence
        Grasp g = new Grasp(ohmm);

        System.out.println("opening gripper...");
        ohmm.armSetGripper(1);
        while (ohmm.armActive());

        System.out.println("lowering arm...");
        g.interpolateArm(0, g.HOME_TO_GROUND);

        System.out.println("extending arm...");
        g.interpolateArm(g.HOME_TO_GRIP, 0);

        System.out.println("closing gripper...");
        ohmm.armSetGripper(0);
        while(ohmm.armActive());

        System.out.println("retracting arm...");
        g.interpolateArm(-g.HOME_TO_GRIP, 0);

        System.out.println("raising arm...");
        g.interpolateArm(0, -g.HOME_TO_GROUND);
    }

    public static void returnToOrigin(OHMMDrive ohmm) {
        /** Storage space for the OHMM pose. **/
        float[] pose = new float[3]; 
        ohmm.driveGetPose(pose);
        //determine angle from orgin to a point
        float desiredAngle = (float) Math.atan2(pose[1], pose[0]);
        //angle transform
        float turnAngle = desiredAngle - pose[2];
        //turn (robot will be facing in opposite direction, must drive backwards to reach origin, add 180 otherwise)
        ohmm.driveTurn(turnAngle);
        //negative one to drive reverse
        float driveDist =  -1 * (float) Math.sqrt(pose[0] * pose[0] + pose[1] * pose[1]);
        ohmm.driveStraight(driveDist);
        float benchmark = 0;
        if (desiredAngle > 3.14f) {
            benchmark = 6;
        }
        ohmm.driveTurn(benchmark - desiredAngle);
        
    }

    public static void main(final String[] args) throws IOException, InterruptedException {
        OHMMDrive ohmm =
          (OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
        if (ohmm == null) System.exit(0);

        Robot.setupOhmm(ohmm);

        // Set up CV Image Server & Thread
        final CvDemo cvd = new CvDemo();
        Thread cvThread = new Thread() {
            public void run() {
                cvd.init(args.length, args);
                cvd.mainLoop();
                cvd.release();
            }
        };
        cvThread.start();

        // Wait for user to click and hit 'r'
        while (!cvd.isReady()) Thread.sleep(100);

        // Create the VW Controller
        int[] target = cvd.getTargetPos();
        target[1] += 20;
        VWController controller = new VWController(target);

        int[] xy = new int[]{-100, -100};

        // Move to target while not at target
        while (!controller.atPickupLocation(xy[0], xy[1])) {
            xy = cvd.getBallPos();
            float[] vw = controller.calcVW(xy[0], xy[1]);
            System.out.printf("v: %f   w: %f\n", vw[0], vw[1]);
            ohmm.driveSetVW(vw[0], vw[1]);
            Thread.sleep(100);
        }
        ohmm.driveSetVW(0f, 0f);

        System.out.println("\nPICKING UP\n");
        Robot.pickup(ohmm);
        Robot.returnToOrigin(ohmm);

        System.out.println("AT ORIGIN");
        cvd.setRunHuh(false);

        ohmm.close();
    }
}
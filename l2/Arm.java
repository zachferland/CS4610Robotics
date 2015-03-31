
package l2;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.math.*;
import java.lang.Math;
import java.util.Arrays;

import java.io.*;

public class Arm {

  //length from joint 0 to joint 1 (mm)
  public static final float len0 = 99;

  //lenght from joint 1 to to joint 2 (wrist) (mm)
  public static final float len1 = 74;




  //inverse kinematics
  //given (Gx, Gz) return (A1,A2, A3)
  public static float[] invK(float gx, float gz) {

    //A1,A2, A3)
    float[] angles = new float[3];

    float beta;
    float wz;
    float sz;
    float wx;
    float sx;
    float rz;
    float rx;

    //differneces in frames of each orgin frame of joint
    wz = 19f;
    wx = 74f;

    sz = 91f;
    sx = 38f;


    rz = gz - wz - sz;
    rx = gx - wx - sx;

    //reach side + calculate this after
    float r = (float)Math.sqrt((rz * rz) + (rx * rx));

    float c1;

    c1 = ((r * r) - (len0 * len0) - (len1 * len1)) / (2 * len0 * len1);


    float s1;

    s1 = (float) Math.sqrt(Math.abs(1 - (c1 * c1)));


    float a1;

    //some float to double conversion issues


    a1 = (float)Math.atan2(s1,c1);


    float alpha;

    //what is rx and rz
    alpha = (float)Math.atan2((-1 * rz), rx);


    float cb;
    float sb;

    //negative 2 for kink down
    cb = ((len1 * len1) - (len0 * len0) - (r * r)) / (-2 * len0 * r);

    sb = (float) Math.sqrt(Math.abs(1 - (cb * cb)));
    // sb = sb * -1;

    beta = (float)Math.atan2(sb,cb);

    float a0;

    a0 = alpha - beta;


    float a2;

    a2 = 0 - a0 - a1;

    angles[0] = a0;
    angles[1] = a1;
    angles[2] = a2;


    return angles;
  }


  public static void setArm(float[] angles, OHMMDrive ohmm) {
    ohmm.armSetAllJointsRad(angles[0], angles[1], angles[2]);
  }


  // public static void driveGoal(float x, float y) {
  //   //make this global var - 280 away.
  //   float r = 280f;

  //   float len = (float) Math.sqrt((x *x) + (y * y));

  //   float m = 1 - (r/len);

  //   float goalx = x * m;

  //   float goaly = y * m;

  //   System.out.println("x  " + goalx);
  //   System.out.println("y  " + goaly);
  // }


  public static void driveToPoint(float x, float y, OHMMDrive ohmm) {
    Arm.turnToPoint(x, y, ohmm);
    float dist = (float) Math.sqrt((x * x) + (y * y));
    float actual = dist - 280f;
    ohmm.driveStraight(actual);
    //driveGoal
    //calcualte distance drive goal, with vertex.
    //ohmm df dist
  }


  public static void turnToPoint(float x, float y, OHMMDrive ohmm) {
    //starts at origin so diff is x y of goal
    float angle =  (float) Math.atan2(y, x);
    ohmm.driveTurn(angle);
  }

  public static void returnToOrigin(float x, float y, OHMMDrive ohmm) {
    ohmm.driveTurn(3.1415926f);
    float dist = (float) Math.sqrt((x * x) + (y * y));
    ohmm.driveStraight(dist);
  }




 //get current pose of of arm







  /**
   * <p>Entry point, see {@link ohmm.OHMM#USAGE}.</p>
   **/
  public static void main(String[] argv) throws IOException {

    //create the OHMM object
    // OHMMDrive ohmm = (OHMMDrive) OHMM.makeOHMM(argv);
    //to use argv for your own purposes, one option is to replace the above
    //call with a hardcoded version like this:
    OHMMDrive ohmm = (OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
    if (ohmm == null) System.exit(0);

    //reset pose

    float x = 1000;
    float y = 300;

    Arm.driveToPoint(x, y, ohmm);

    while (ohmm.driveGetQueue() != 0) {
        
    }


    ohmm.armEnable(true);
    
    float[] ik = Arm.invK(220f, 45f);



    // Arm.setArm(ik, ohmm);
    ohmm.armSetAllJointsRad(ik[0], ik[1], ik[2]);
    while (ohmm.armActive()) {
        
    }

     ohmm.armSetGripper(1.0f);
    while (ohmm.armActive()) {
        
    }

    ik = Arm.invK(270f, 45f);
    ohmm.armSetAllJointsRad(ik[0], ik[1], ik[2]);
     while (ohmm.armActive()) {
        
    }


    ohmm.armSetGripper(0.0f);

    while (ohmm.armActive()) {
        
    }

    ohmm.armHome();

    while (ohmm.armActive()) {
        
    }

    Arm.returnToOrigin(x, y, ohmm);

    while (ohmm.driveGetQueue() != 0) {
        
    }

    System.out.println(ik[0]);
    System.out.println(ik[1]);
    System.out.println(ik[2]);
    //ohmm.armEnable(false);


    // OHMMShell shell = new OHMMShell(ohmm);
    // shell.readEvalPrintLoop();

    ohmm.close();

  }
}
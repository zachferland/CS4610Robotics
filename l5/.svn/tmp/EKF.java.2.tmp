/*******************************************************************************
* CS4610: Lab 5
* Group 1: Nicholas Jones, Zach Ferland
*******************************************************************************/
package l5;

import java.io.IOException;

import ohmm.*;
import ohmm.Grasp.*;
import Jama.*;
import april.tag.*;
import Jama.EigenvalueDecomposition;
import java.lang.Math;

public class EKF {
    //semantics, array access
    static final int X = 0;
    static final int Y = 1;
    static final int T = 2;
    static final int LEFT = 0;
    static final int RIGHT = 1;

    //robot baseline (mm)
    public static final double b = 205;
    //radius of robot wheels (mm)
    public static final double r = 39;

    // Robot Pose
    protected float[] pose = new float[3];
    // Wheel positions (rotations) (total)
    protected float[] motPos = new float [2];
    // covariance matrix of pose - initialized as a zero matrix
    Matrix covariancePose = new Matrix(3,3);

    // EKF needs an instance of OHMMDrive
    OHMMDrive ohmm = null;

    //initialize EKF
    public EKF(OHMMDrive ohmm) { 
        this.ohmm = ohmm;
    }

    // Main run function when used in other objects or classes, run in a seperate thread
    public void run() {
        long taskTime = 0;
        long sleepTime = 1000/1;
        while (true) {
            taskTime = System.currentTimeMillis();

            predictionProcess();
            //errorEllipse();
            // poseUncertainty(1);

            // if the area of the error elipse gets too large from odometry, run a update process
            double areaUncert = 600;
            if (errorSize() > areaUncert) { 
                updateProcess();
            }

            taskTime = System.currentTimeMillis()-taskTime;
            if (sleepTime-taskTime > 0 ) {
            try { 
                    Thread.sleep(sleepTime-taskTime);
            } catch(InterruptedException e) { } 
          }
        }
    }

    // runs at (times per second) determines odometry with uncertainty
    public void predictionProcess() {
        System.out.println("///////////////////////////////////////////////////////////////////////");

        //get current robot pose - x y t
        float[] newPose = new float[3];
        ohmm.driveGetPose(newPose);

        System.out.println("NEW POSE = X - " + newPose[X] + " Y - " + newPose[T] + " T - " + newPose[T]);

        //get wheel rotations since last update - 1 = left, 2 = right
        float[] newMotPos = new float[2];
        ohmm.motGetPos(newMotPos);

        //calculate difference since last time
        float motPosLDiff = newMotPos[LEFT] - motPos[LEFT];
        float motPosRDiff = newMotPos[RIGHT] - motPos[RIGHT];
        float wl = motPosLDiff;
        float wr = motPosRDiff;

        //gurantee difference is not zero, zero difference causes division/multiplications issues
        //when calculating pose covariance
        if (wl == wr) {
             wr = wr + 0.00000001f;
        }

        //Create diagnol covariance matrix for wheels
        Matrix covarianceWheels = new Matrix(2,2);

        //set standard deviation of wheels (will have to determine this with tests)
        float sdwheel = 0.01f;

        //set proportional standard deviation or fixed constant as diagonal matrix
        covarianceWheels.set(0,0,sdwheel * wl);
        covarianceWheels.set(1,1,sdwheel * wr);

        // Print Pose Covariance
        System.out.println("Pose Covariance");
        covariancePose.print(2,2);

        //Initialize 2D Jacobian Array of Pose ()
        double[][] jacobianPoseArray = new double[3][3];

        //calculate jacobian array of partial derivatives of F over p
        jacobianPoseArray[0][0] = 1;
        jacobianPoseArray[0][1] = 0;
        jacobianPoseArray[0][2] = ((b*(wr+wl))/(2*(wr-wl)))*(-Math.cos(pose[T])+Math.cos(pose[T]+(r/b)*(wr-wl)));
        jacobianPoseArray[1][0] = 0;
        jacobianPoseArray[1][1] = 1;
        jacobianPoseArray[1][2] = ((b*(wr+wl))/(2*(wr-wl)))*(-Math.sin(pose[T])+Math.sin(pose[T]+(r/b)*(wr-wl)));
        jacobianPoseArray[2][0] = 0;
        jacobianPoseArray[2][1] = 0;
        jacobianPoseArray[2][2] = 1;

        //create matrix from 2D array
        Matrix jacobianPose = new Matrix(jacobianPoseArray);

        // System.out.println("Jacobian Pose");
        // jacobianPose.print(3,2);

        //transpose of jacobian pose
        Matrix jacobianPoseT = jacobianPose.transpose();

       // System.out.println("Jacobian Pose Transpose");
       // jacobianPoseT.print(3,2);

       //Initialize 2D Jacobian Array of wheels ()
       double[][] jacobianWheelArray = new double[3][2];

        //calculate jacobian array of partial derivatives of F over w
        jacobianWheelArray[0][0] = ((b/2)*((wr+wl)/Math.pow(wr-wl, 2))*(-Math.sin(pose[T])+Math.sin(pose[T]+(r/b)*(wr-wl)))) + ((b/2)*(1/(wr-wl))*(-Math.sin(pose[T])+Math.sin(pose[T]+(r/b)*(wr-wl)))) - ((r/2)*((wr+wl)/(wr-wl))*(Math.cos(pose[T]+(r/b)*(wr-wl))));
        jacobianWheelArray[0][1] = ((-b/2)*((wr+wl)/Math.pow(wr-wl, 2))*(-Math.sin(pose[T])+Math.sin(pose[T]+(r/b)*(wr-wl)))) + ((b/2)*(1/(wr-wl))*(-Math.sin(pose[T])+Math.sin(pose[T]+(r/b)*(wr-wl)))) + ((r/2)*((wr+wl)/(wr-wl))*(Math.cos(pose[T]+(r/b)*(wr-wl))));
        jacobianWheelArray[1][0] = ((r/2)*((wr+wl)/(wr-wl))*(Math.sin(pose[T]+(r/b)*(wr-wl))))-((b/2)*((wr+wl)/Math.pow(wr-wl, 2))*(Math.cos(pose[T])-Math.cos(pose[T]+(r/b)*(wr-wl)))) + ((b/2)*(1/(wr-wl))*(Math.cos(pose[T])-Math.cos(pose[T]+(r/b)*(wr-wl))));
        jacobianWheelArray[1][1] = ((-r/2)*((wr+wl)/(wr-wl))*(Math.sin(pose[T]+(r/b)*(wr-wl)))) + ((b/2)*((wr+wl)/Math.pow(wr-wl, 2))*(Math.cos(pose[T])-Math.cos(pose[T]+(r/b)*(wr-wl)))) + ((b/2)*(1/(wr-wl))*(Math.cos(pose[T])-Math.cos(pose[T]+(r/b)*(wr-wl))));
        jacobianWheelArray[2][0] = -r/b;
        jacobianWheelArray[2][0] = r/b;

        //create matrix from array
        Matrix jacobianWheel = new Matrix(jacobianWheelArray);

        //  System.out.println("Jacobian Wheels");
        // jacobianWheel.print(2,2);

        //transpose of jacobian wheel
        Matrix jacobianWheelT = jacobianWheel.transpose();

        // System.out.println("Jacobian Wheels Transpose");
        //  jacobianWheelT.print(3,2);

        //Calculate new covariance matrix of pose
        // newsigmaP = (Jp * sigmaP * Jpt) + (Jw * sigmaW * Jwt)
        Matrix newCovariancePose = new Matrix(3,3);

        //pose prediction error (Jp * sigmaP * Jpt) 
        Matrix tempP = jacobianPose.times(covariancePose);
        Matrix errorPose = tempP.times(jacobianPoseT);

        //wheel error
        Matrix tempW = jacobianWheel.times(covarianceWheels);
        Matrix errorWheel = tempW.times(jacobianWheelT);

        newCovariancePose = errorPose.plus(errorWheel);

        //Update values with new calculations
        pose = newPose;
        motPos = newMotPos;
        covariancePose = newCovariancePose;
    }

    public void updateProcess() {
        // System.out.println("Covariance Before");
        // covariancePose.print(3,2);

        System.out.println("UPDATE PROCESS ------------------------------------------");

        // Create diagonal uncertainty matrix from exteroceptive sensing (q)
        Matrix covarianceVisQ = new Matrix(3,3);
        // accuracy here is unknown and unmeasured, used below is a best guess
        // 10 mm
        double sdpose = 2;
        double sdtheta = 0;
        //set  standard deviation diagonal matrix
        covarianceVisQ.set(0,0,sdpose);
        covarianceVisQ.set(1,1,sdpose);
        covarianceVisQ.set(2,2,sdtheta);

        //get visual sensing pose
        float[] poseQ = new float[3];
        // SHOULD GET POSE FROM EXTROCEPTIVE SENSING CLASS HERE
        // FOR THE TIME BEING THIS IS FAKED
        // MarkerDetection.getPose(poseQ);
         ohmm.drivePause();
        // messy fake pause
        try { 
            Thread.sleep(3000); 

        } catch (InterruptedException ie) { 
           
        } 

        ohmm.driveUnPause();

        // Set given pose of extroceptive sensors (this is fake for the time being as stated above)
        Matrix mPoseQ = new Matrix(3,3);
        mPoseQ.set(0,0, pose[X] + 1);
        mPoseQ.set(1,1, pose[Y] - 1);
        mPoseQ.set(2,2, pose[T]);

        //the addition of covariance from vis sensing and odometry
        Matrix covarianceV =  covarianceVisQ.plus(covariancePose);

        // System.out.println("COVARIANCE V");
        // covarianceV.print(3,2);

        //instantiate corrected pose matrix 3 X 3 diagnol
        Matrix newPose = new Matrix(3,3);

        //inverse of matrix covarianceV
        Matrix covarianceVI = covarianceV.inverse();

        //kalman gain
        Matrix kgain = covariancePose.times(covarianceVI);

        // System.out.println("KGAIN");
        // kgain.print(3,2);

        // covert current pose to matrix - complete explicitly since it is not a double array
        // 3x3 diagonal matrix
        Matrix poseM = new Matrix(3, 3);
        poseM.set(0,0,pose[X]);
        poseM.set(1,1,pose[Y]);
        poseM.set(2,2,pose[T]);

        // v = q - p
        Matrix poseV = mPoseQ.minus(poseM);

        // System.out.println("POSE V");
        // poseV.print(3,2);

        //update pose by weighted uncertainty from both vis and odometry 
        // p1 = p + Kv
        newPose = poseM.plus(kgain.times(poseV));

        // System.out.println("NEW POSE");
        // newPose.print(3,2);

        //update pose covariance, decrease uncertainty 
        Matrix newCovariancePose = new Matrix(3,3);

        //tranpose kalman gain
         Matrix kgainT = kgain.transpose();

        // System.out.println("KGAIN TRANSPOSE");
        // kgainT.print(3,2);

         //ksigma * Vsigma * ksigmaT
         Matrix temp = kgainT.times(kgain.times(covarianceV));

        // System.out.println("MINUS COVARIANCE");
        // temp.print(3,2);

        newCovariancePose = covariancePose.minus(temp);

        // System.out.println("NEW POSE COVARIANCE");
        // newCovariancePose.print(3,2);

         //update pose and covariance 
         pose[X] = (float) newPose.get(0,0);
         pose[Y] = (float) newPose.get(1,1);
         pose[T] = (float) newPose.get(2,2);
          System.out.println("NEW POSE = X - " + pose[X] + " Y - " + pose[T] + " T - " + pose[T]);

         // SHOULD ONLY THE DIAGNOLS BE PASSED ALONG?
         // had many issues here with the covariance growing too large too fast, after this 
        //calculation, and negatives. Also turning error grew to large, set fixed for now
         newCovariancePose.set(0,1,0);
         newCovariancePose.set(0,2,0);
         newCovariancePose.set(1,0,0);
         newCovariancePose.set(1,2,0);
         newCovariancePose.set(2,0,0);
         newCovariancePose.set(2,1,0);
         // newCovariancePose.set(2,2,Math.abs(newCovariancePose.get(2,2)));
         newCovariancePose.set(2,2,0.01);

         covariancePose = newCovariancePose;
         //update pose when sensing data is more accurate
         // ohmm.driveSetPose(pose[X], pose[Y], pose[T])

        System.out.println("New Pose Covariance");
        covariancePose.print(3,2);

        System.out.println("END UPDATE PROCESS ------------------------------------------");
    }

    //get error ellipse (returns radi r0 and r1 in array) 
    //TODO - add functionality to return all need information to draw ellipse
    public double[] errorEllipse(){
        //o (will need these to draw ellipse, the xy of pose)
        // maybe return these after
        float x = pose[X];
        float y = pose[Y];

        //get eigenValues
        double[] eVal = getEigenVal();

        //desired uncertainty
        double p = 95;

        double r0 = Math.sqrt(-2*Math.log(1-p/100)*eVal[0]);
        double r1 = Math.sqrt(-2*Math.log(1-p/100)*eVal[1]);

        double[] radi = new double[2];
        radi[0] = r0;
        radi[1] = r1;

        double area = radi[0] * radi[1] * Math.PI;

        System.out.println("----------------------------------------");
        System.out.println("ERROR ELLIPSE AREA - " + area + " R1 = " + radi[0] + " R2 = " + radi[1]);
        System.out.println("----------------------------------------");

        return radi;
    }

    //area of current error ellipse, and singe value that can be used to compare errors
    public double errorSize() {
        double[] ellipseRadi = errorEllipse();
        double area = ellipseRadi[0] * ellipseRadi[1] * Math.PI; 
        return area;
    }

    //get pose uncertainty based on a fixed ellipse area
    // does not work well yet
    public double poseUncertainty(double area){
        //get eigenValues
        double[] eVal = getEigenVal();

        double raised = Math.sqrt(Math.pow((area/Math.PI),2)/(4*eVal[0]*eVal[1]));
        double p = 100 * (Math.pow(Math.E, raised) - 1);

        //1 or 100?
        if (p > 100) {
            p = 100;
        }

        System.out.println("Uncertainty Percent - " + p);       
        return p;
    }

    //returns two eigen values in double array based on current pose
    public double[] getEigenVal(){
        //take upper left 2x2 submatrix from the 3x3 pose covariance matrix
        Matrix e = covariancePose.getMatrix(0,1,0,1);

        //compute an Eigendecomposition of E
        EigenvalueDecomposition eigenDecomp = new EigenvalueDecomposition(e);

        // diagonal matrix, two values
        Matrix eigenValues = eigenDecomp.getD();

        //EigenValues l0 and l1
        double l0 = eigenValues.get(0,0);
        double l1 = eigenValues.get(1,1);

        //create return array
        double[] eVal = new double[2];
        eVal[0] = l0;
        eVal[1] = l1;

        return eVal;  
    }

    //Main run class for EKF if not used with another class or object, simple a demonstration
    //of EKF while the robot continously drives in a straight line
    public static void main(final String[] args) throws IOException, InterruptedException {
       OHMMDrive ohmm =
                (OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
        if (ohmm == null) System.exit(0);

        // reset pose 
        ohmm.motReinit();
        
        EKF ekf = new EKF(ohmm);

        ohmm.driveResetPose();

        ohmm.driveSetVW(40f, 0.0f);

        //run n times per second (note - handle this better)
        long n = 1;
        long taskTime = 0;
        long sleepTime = 1000/n;
        while (true) {
            taskTime = System.currentTimeMillis();

            ekf.predictionProcess();
            // ekf.errorEllipse();
            ekf.poseUncertainty(100);

            // if error ellipse area get too large, run an update process 
            double areaUncert = 600;
            if (ekf.errorSize() > areaUncert) { 
                ekf.updateProcess();
            }

          taskTime = System.currentTimeMillis()-taskTime;
          if (sleepTime-taskTime > 0 ) {
            Thread.sleep(sleepTime-taskTime);
          }
        }

        //ohmm.close();
    }
}

/*******************************************************************************
*
* TagLocation
* CS4610: Robotics Science and Systems
* Lab 5
* 4/14/2014
* Nicholas Jones, Zach Ferland
*
*******************************************************************************/

package l5;

import ohmm.*;

import com.googlecode.javacpp.*;
import com.googlecode.javacv.*;
import com.googlecode.javacv.cpp.*;
import static com.googlecode.javacv.cpp.opencv_calib3d.*;
import static com.googlecode.javacv.cpp.opencv_contrib.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_features2d.*;
import static com.googlecode.javacv.cpp.opencv_flann.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_legacy.*;
import static com.googlecode.javacv.cpp.opencv_ml.*;
import static com.googlecode.javacv.cpp.opencv_objdetect.*;
import static com.googlecode.javacv.cpp.opencv_video.*;

import java.awt.image.BufferedImage;
import java.awt.Graphics2D;
import java.awt.Color;

import java.io.IOException;

import april.tag.TagDetector;
import april.tag.TagDetection;
import april.tag.CameraUtil;
import april.tag.Tag36h11;
import april.tag.TagFamily;

import april.jmat.LinAlg;

import java.util.ArrayList;
import java.util.Arrays;

/**
* FiducialTagFinder runs an image server that serves the images from the video
* camera.  It also takes the images from the camera and processes them with 
* AprilTags to find the 3D location of the tags in the picure.  This information
* can be used to obtain the location of the robot in the world frame.
* @author Nicholas Jones
* @version 4/14/2014
*/
public class FiducialTagFinder extends CvBase implements Runnable {
    // CAMERA CONSTANTS
    public static final double[] CAMERA_F
        = new double[] {534.816842, 531.883274}; // Camera focal length

    public static final double[] CAMERA_C
        = new double[] {304.638279, 224.403908}; // Camera Center

    public static final double[] CAMERA_K
        = new double[] {0.0403554, -0.022707, -0.014288, -0.263265}; // Camera K

    public static final double TAG_SIZE = 0.125; // Size of tag in meters

    // IMAGE SERVER FIELDS
    public static final int DEF_MAX_FPS = 5; // Max frames per second
    public static final String APPNAME = "TagFinder"; // Name of APP


    private ArrayList<TagDetection> currentlySeenTags;
    private TagDetector detector;
    private WorldMap map;
    protected IplImage procImg = null;



    /**
    * Constructor
    * @param family - The TagFamily which all tags seen will belong to.
    * @param map - The path of the file containing the map of the tags in the 
    *              world frame.
    * @throws IOException
    */
    public FiducialTagFinder(TagFamily family, String map) throws IOException {
        super(APPNAME);
        maxFPS = DEF_MAX_FPS;

        this.map = new WorldMap(map);

        System.out.println(this.map);

        this.currentlySeenTags = new ArrayList<TagDetection>();
        this.detector =  new TagDetector(family);
    }


    /**
    * Start the thread.
    */
    public void run() {
        mainLoop();
        release();
    }

    /**
    * Process the frame from the camera to find the TagDetections.
    * @param frame - The image from the camera
    * @return The image from the camera unchanged
    */
    public IplImage process(IplImage frame) {

        if (frame != null)
            this.currentlySeenTags = this.detector.process(frame.getBufferedImage(), CAMERA_C);

        return frame;
    }

    /**
    * Releases allocated memory.
    */
    public void release() { 
        if (procImg != null) { procImg.release(); procImg = null; }
            super.release();
    }


    /**
    * Get the locations of the visible tags.
    * @return A list of TagLocations containing the 3D location of the tags in
    *         the robot frame.
    */
    private ArrayList<TagLocation> getVisibleTagLocs() {
        ArrayList<TagLocation> list = new ArrayList<TagLocation>();

        for (TagDetection td : this.currentlySeenTags) {
            double m[][] = CameraUtil.homographyToPose(CAMERA_F[0],
                                                       CAMERA_F[1],
                                                       CAMERA_C[0],
                                                       CAMERA_C[1],
                                                       td.homography);
            m = CameraUtil.scalePose(m, 2.0, TAG_SIZE);

            for (int i = 0; i < m.length; i++) {
                for (int j = 0; j < m[0].length; j++)
                    System.out.printf("%8.3f ", m[i][j]);
                System.out.println();
            }

            //System.out.println();
            list.add(new TagLocation(td.id, m[0][3], m[1][3], m[2][3], 0.0 /* TODO */));
        }

        System.out.println("------------------------");
        return list;
    }

    /**
    * Find the robot location in the world frame using the tags visible together
    * with the world map.
    * @return {x, y, z, rotation} of the robot in meters and radians 
    */
    public double[] getRobotLocation() {
        ArrayList<TagLocation> tags = getVisibleTagLocs();

        if (tags.size() == 0)
            return null;
        else
            return this.map.findRobotGlobal(tags.get(0));
    }


    /**
    * ENTRY POINT
    */
    public static void main(String[] args) throws IOException {
        FiducialTagFinder finder = new FiducialTagFinder(new Tag36h11(), "tagmap.map");
        finder.init(args.length, args);

        new Thread(finder).start();

        for (int i = 0; i < 6000; i++) {
            double[] robot = finder.getRobotLocation();
            if (robot != null)
                System.out.println(Arrays.toString(robot));
            try {Thread.sleep(1000);}
            catch (InterruptedException e) {}
        }
    }
}
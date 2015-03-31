/*******************************************************************************
*
* WorldMap
* CS4610: Robotics Science and Systems
* Lab 5
* 4/14/2014
* Nicholas Jones, Zach Ferland
*
*******************************************************************************/

package l5;

import java.util.Scanner;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

/**
* WorldMaps reads a file containing a list of tag locations and provides a
* method for finding the location of the robot.
* @author Nicholas Jones
* @version 4/14/2014
*/
public class WorldMap {
    private ArrayList<TagLocation> tagList;

    /**
    * Constructor
    * Reads in the Tag information from the file
    * @param url - The path of the file containing the map information.
    */
    public WorldMap(String url) throws IOException {
        this.tagList = new ArrayList<TagLocation>();

        Scanner scnr = new Scanner(new File(url));

        while (scnr.hasNext()) {
            Scanner line = new Scanner(scnr.nextLine());
            // TAG_ID TAG_X TAG_Y TAG ROTATION
            this.tagList.add(new TagLocation(line.nextInt(),
                                             line.nextDouble(),
                                             line.nextDouble(),
                                             0.0,
                                             line.nextDouble()));
        }
    }


    /**
    * Find the robot location in the world frame by comparing a tag location in 
    * the robot frame to its location in the world frame.
    * @param tl - The TagLocation for a tag seen by the robot
    * @return {x, y, z, theta} of robot in world frame in meters and radians
    */
    public double[] findRobotGlobal(TagLocation tl) {

        for (TagLocation info : this.tagList) {
            if (info.id == tl.id) {
                System.out.printf("Found %d in map\n", info.id);
                double[] robot = new double[3];

                // TODO
                robot[0] = tl.x / .125;
                robot[1] = tl.y / .125;
                robot[2] = tl.rotation / .125;

                return robot;
            }
        }

        return null;
    }

    /**
    * @return A String representing the World Map
    */
    public String toString() {
        String s = "";

        for (TagLocation info : this.tagList) {
            s += info + "\n";
        }

        return s;
    }
}
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

/**
* TagLocation is a class to contain information about the 3D location and
* rotation (around the axis orthogonal to the face of the tag.)
* @author Nicholas Jones
* @version 4/14/2014
*/
public class TagLocation {
    /** The id of the tag */
    public int id;

    /** The x coordinate of the tag in meters. */
    public double x;

    /** The y coordinate of the tag in meters. */
    public double y;

    /** The z coordinate of the tag in meters. */
    public double z;

    /** The rotation of the tag around the axis orthogonal to its face in rad */
    public double rotation;

    /**
    * Constructor
    * @param id - The id of the tag
    * @param x - The x coordinate of the tag in meters
    * @param y - The y coordinate of the tag in meters
    * @param z - The z coorfinate of the tag in meters
    * @param rotation - The rotation of the tag around the axis orthogonal to 
    *                   its face in rad
    */
    public TagLocation(int id, double x, double y, double z, double rotation) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.rotation = rotation;
    }


    /**
    * @return A String representing the TagLocation
    */
    @Override
    public String toString() {
        return  "id: "   + this.id +
                " x: "   + this.x  +
                " y: "   + this.y  +
                " z: "   + this.z  + 
                " rot: " + this.rotation;
    }
}
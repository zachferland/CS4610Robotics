/**
 * <p>OHMMShell Demo.</p>
 *
 * <p>This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.</p>
 *
 * <p>This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.</p>
 *
 * <p>You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA</p>
 *
 * <p>Copyright (c) 2011 Marsette Vona</p>
 **/

package l2;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.io.*;

/**
 * <p>OHMM JScheme command shell ({@link ohmm.OHMMShell}) demonstration.</p>
 *
 * @author Marsette Vona
 **/
public class OHMMShellDemo {

  private static final String svnid =
  "$Id: OHMMShellDemo.java 1255 2013-02-21 19:45:47Z vona $";

  /**
   * <p>Entry point, see {@link ohmm.OHMM#USAGE}.</p>
   **/
  public static void main(String[] argv) throws IOException {

    //create the OHMM object
    OHMMDrive ohmm = (OHMMDrive) OHMM.makeOHMM(argv);
    //to use argv for your own purposes, one option is to replace the above
    //call with a hardcoded version like this:
    //OHMMDrive ohmm =
    //  (OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});

    if (ohmm == null) System.exit(0);

    //If you want to run an instance of the OHMM HTTP image server to show
    //debug images, you could start it here.
    //
    //ImageServerDemo imageServer = new ImageServerDemo(ohmm);
    //imageServer.start();

    //to use this file as a template:
    //
    //1 - rename the file to YourClass.java
    //
    //2 - change the class name above from OHMMShellDemo to YourClass
    //
    //3 - change the author info and other documentation above
    //
    //4 - comment out the next two lines that run an OHMMShell
    //
    //5 - write whatever code you want here that makes use of the ohmm object
    OHMMShell shell = new OHMMShell(ohmm);
    shell.readEvalPrintLoop();

    ohmm.close();
  }
}

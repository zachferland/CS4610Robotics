package l5;

import ohmm.*;
import java.io.IOException;

public class Stop {
	public static void main(String[] args) throws IOException, InterruptedException {
        OHMMDrive ohmm =
          (OHMMDrive) OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
        if (ohmm == null) System.exit(0);

		ohmm.driveSetVW(0f, 0f);

        ohmm.close();
	}
}
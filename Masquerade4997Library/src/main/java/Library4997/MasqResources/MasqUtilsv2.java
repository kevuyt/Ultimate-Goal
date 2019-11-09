package Library4997.MasqResources;

import Library4997.MasqControlSystems.MasqPID.MasqPIDConstants;
import Library4997.MasqResources.MasqHelpers.API_KEYS;

/**
 * Created by Keval Kataria on 10/11/2019
 */
public class MasqUtilsv2 implements API_KEYS {
    private static  MasqPIDConstants turnConstants;
    private static MasqPIDConstants driveConstants;

    public static void setTurnConstants(int kP, int kI, int kD) {
        turnConstants = new MasqPIDConstants(kP, kI, kD);
    }
    public static void setDriveConstants(int kP, int kI, int kD) {
        driveConstants = new MasqPIDConstants(kP, kI, kD);
    }

    public static MasqPIDConstants getTurnConstants() {
        return turnConstants;
    }

    public static MasqPIDConstants getDriveConstants() {
        return driveConstants;
    }
}
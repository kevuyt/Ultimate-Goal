package Library4997.MasqResources;

import Library4997.MasqControlSystems.MasqPID.MasqPIDConstants;

/**
 * Created by Keval Kataria on 10/11/2019
 */
public class MasqUtilsv2 {
    private static  MasqPIDConstants turnConstants;
    private static MasqPIDConstants driveConstants;
    private static MasqPIDConstants velocityConstants;
    private static MasqPIDConstants angleConstants;

    public static void setTurnConstants(double kP, double kI, double kD) {turnConstants = new MasqPIDConstants(kP, kI, kD);}
    public static void setDriveConstants(double kP, double kI, double kD) {driveConstants = new MasqPIDConstants(kP, kI, kD);}
    public static void setVelocityConstants(double kP, double kI, double kD) {velocityConstants = new MasqPIDConstants(kP, kI, kD);}
    public static void setAngleConstants(double kP, double kI, double kD) {angleConstants = new MasqPIDConstants(kP, kI, kD);}

    public static MasqPIDConstants getTurnConstants() {return turnConstants;}
    public static MasqPIDConstants getDriveConstants() {return driveConstants;}
    public static MasqPIDConstants getVelocityConstants() {return velocityConstants;}
    public static MasqPIDConstants getAngleConstants() {return angleConstants;}
}
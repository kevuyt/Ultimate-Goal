package Library4997.MasqUtilities;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;


/**
 * Created by Archish on 10/16/17.
 */

public class MasqUtils implements API_KEYS {
    public static MasqLinearOpMode linearOpMode;
    private static MasqClock clock = new MasqClock();
    public static final double NEVERREST_ORBITAL_20_RPM = 340;
    public static final double NEVERREST_ORBITAL_20_TICKS_PER_ROTATION = 537.6;
    public static final double wheelDiameter = 4;
    public static final double gearRatio = 1;
    public static final double CLICKS_PER_INCH = (NEVERREST_ORBITAL_20_TICKS_PER_ROTATION / (wheelDiameter * Math.PI)) * gearRatio;
    private static double integral = 0;
    private static double prevError = 0;
    public static final int DEFAULT_SLEEP_TIME = 0;
    public static final double DEFAULT_TIMEOUT = 1;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = API_KEYS.VUFORIA_KEY;

    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void sleep (double sleep) {
        try {Thread.sleep((long) sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void setLinearOpMode(MasqLinearOpMode pLinearOpMode) {
        linearOpMode = pLinearOpMode;
    }
    public static Telemetry getTelemetry() {
        return linearOpMode.telemetry;
    }
    public static boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }
    public HardwareMap getHardwareMap() {
        return linearOpMode.hardwareMap;
    }
    public class KP {
        public static final double TURN = +0.058;
        public static final double DRIVE_ANGULAR = +0.05;
        public static final double DRIVE_ENCODER = .7;
        public static final double GO_ENCODER = .9;
        public static final double MOTOR_TELEOP = +0.1;
        public static final double MOTOR_AUTONOMOUS = +0.06;
    }
    public class KI {
        public static final double TURN = +0.00;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class KD {
        public static final double TURN = +0.0;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.001;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class ID {
        public static final double TURN = +1.0;
        public static final double DRIVE = +1.0;
        public static final double MOTOR_TELEOP = +1.0;
        public static final double MOTOR_AUTONOMOUS = +1.00;
    }
    public static class VuMark {
        public static final boolean isCenter(String vuMark) {return vuMark.toLowerCase().contains("c");}
        public static final boolean isLeft(String vuMark) {return vuMark.toLowerCase().contains("l");}
        public static final boolean isRight(String vuMark) {return vuMark.toLowerCase().contains("g");}
        public static final boolean isUnKnown (String vuMark) {return vuMark.toLowerCase().contains("u");}
    }
}

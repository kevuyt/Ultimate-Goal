package Library4997.MasqResources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Library4997.MasqResources.MasqHelpers.API_KEYS;
import Library4997.MasqWrappers.MasqLinearOpMode;


/**
 * Created by Archish on 10/16/17.
 */

public class MasqUtils implements API_KEYS {
    private static MasqLinearOpMode linearOpMode;
    public static final double MECH_DRIVE_MULTIPLIER = 1.4;
    public static final double MECH_ROTATION_MULTIPLIER = 0.4;
    public static final int DEFAULT_SLEEP_TIME = 0;
    public static final double DEFAULT_TIMEOUT = 2;
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
    public static MasqLinearOpMode getLinearOpMode () {
        return linearOpMode;
    }
    public static boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }
    public HardwareMap getHardwareMap() {
        return linearOpMode.hardwareMap;
    }
    public class KP {
        public static final double TURN = +0.025;
        public static final double TURN_POM = -0.1;
        public static final double DRIVE_ANGULAR = +0.1;
        public static final double DRIVE_ENCODER = 2.5;
        public static final double PATH = .01;
        public static final double MOTOR_TELEOP = +0.005;
        public static final double MOTOR_AUTONOMOUS = +0.001;
    }
    public class KI {
        public static final double PATH = 0.0;
        public static final double TURN = +0.0;
        public static final double TURN_POM = 0.000005;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.000;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class KD {
        public static final double PATH = .0;
        public static final double TURN = +0.0;
        public static final double TURN_POM = -0.00;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.000;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class ID {
        public static final double TURN = +1.0;
        public static final double DRIVE = +1.0;
        public static final double MOTOR_TELEOP = +1.0;
        public static final double MOTOR_AUTONOMOUS = +1.00;
    }
    public static double max(double... vals) {
        double max = Double.MIN_VALUE;
        for (double d: vals) if (max < d) max = d;
        return max;
    }
    public static double min(double... vals) {
        double min = Double.MAX_VALUE;
        for (double d: vals) if (min > d) min = d;
        return min;
    }
    public double lowPass (double upperThresh, double value, double prev) {
        if (value < upperThresh) return prev;
        return value;
    }
}

package Library4997.MasqResources;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqWrappers.MasqLinearOpMode;


/**
 * Created by Archish on 10/16/17.
 */

public class MasqUtils {
    private static MasqLinearOpMode linearOpMode;
    public static final double MECH_DRIVE_MULTIPLIER = 1.4;
    public static final double MECH_ROTATION_MULTIPLIER = 0.4;
    public static final double DEFAULT_SLEEP_TIME = 0.5;
    public static final double DEFAULT_TIMEOUT = 2;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = "AT47cqv/////AAABmfWDhjR9GUP+p3V+yVCiSZE6RH3" +
            "KNyZpyijp6yi/cAQt+p5stWYPhiE0/oQ1v4HK9S6Y6JiCkmnWR5PN8rl" +
            "xIgZXTZi5F3clx9w9LzsUfEhz2Ctt0E5a6Rss8uiHgZHEr+ZclXe4meX" +
            "Tq0CPHfSlQNlWi6/KZJWnueUPLOkvMi48J9fPAp2xXrliVRQ3Zs0gWHj" +
            "6/iH7SwxefH4aDwv4aOG9amOB+pqD0AZeBzeuQzjl5gjwDVZNchs8muA" +
            "yAnqK/wrtoJ9gFWXlJ5wK1hzMnP3+pO+uJl3hU/3LF9tzsL60nZkxL0r" +
            "zD+fIy0fi8xx1LfysN2URrT82AtUQ2teoPQRFFsgVmYii/W6/1ZUJKcwH";

    public static void sleep (int milliSeconds) {
        try {Thread.sleep(milliSeconds);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void sleep (double sleep) {
        try {Thread.sleep((long) sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void setLinearOpMode(MasqLinearOpMode pLinearOpMode) {
        linearOpMode = pLinearOpMode;
    }

    public static double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle = 360;
        return angle;
    }

    public static boolean tolerance(double value1, double value2, double tolerance) {
        return Math.abs(value1 - value2) < tolerance;
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
    /*public class KP {
        public static final double TURN = 0.015;
        public static final double TURN_POM = -0.1;
        public static final double ANGLE = 0.005;
        public static final double DRIVE = 3;
        public static final double PATH = .01;
        public static final double VELOCITY_TELE = 0.002;
        public static final double VELOCITY_AUTO = 0.002;
    }
    public class KI {
        public static final double PATH = 0.0;
        public static final double TURN = 0.0;
        public static final double ANGLE = 0.000;
        public static final double TURN_POM = 0.000005;
        public static final double DRIVE = 0.0;
        public static final double VELOCITY_TELE = 0.000;
        public static final double VELOCITY_AUTO = 0.002;
    }
    public class KD {
        public static final double PATH = .0;
        public static final double TURN = 0.0;
        public static final double ANGLE = 0.000;
        public static final double TURN_POM = -0.00;
        public static final double DRIVE = 0.0;
        public static final double VELOCITY_TELE = 0.000;
        public static final double VELOCITY_AUTO = 0.002;
    }
    public class ID {
        public static final double TURN = 1.0;
        public static final double DRIVE = 1.0;
        public static final double MOTOR_TELEOP = 1.0;
        public static final double MOTOR_AUTONOMOUS = 1.00;
    }*/
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
    public static void toggle(boolean button, MasqServo servo, double prevPos) {
        if (MasqUtils.tolerance(servo.getPosition(), prevPos,0.01) && button) {
            if (MasqUtils.tolerance(servo.getPosition(), 0, 0.01)) servo.setPosition(1);
            else if (MasqUtils.tolerance(servo.getPosition(), 1, 0.01)) servo.setPosition(0);
        }
    }
    public static void toggle(boolean button, MasqServoSystem servoSystem, double prevPos) {
        for (MasqServo servo : servoSystem.servos) {
            toggle(button, servo, prevPos);
        }
    }
    public static void toggle(boolean button, MasqServo servo, double prevPos, double pos1, double pos2) {
        if (MasqUtils.tolerance(servo.getPosition(), prevPos, 0.01) && button) {
            if (MasqUtils.tolerance(servo.getPosition(), pos1, 0.01)) servo.setPosition(pos2);
            else if (MasqUtils.tolerance(servo.getPosition(), pos2, 0.01)) servo.setPosition(pos1);
        }
    }
    public static void toggle (boolean button, MasqServo servo, double prePos, Runnable action) {
        toggle(button, servo, prePos);
        if (button) {
            Thread thread = new Thread(action);
            thread.start();
        }
    }
    public void toggle (boolean button, double current, double prev, double value1, double value2, Runnable action1, Runnable action2) {
        if (button && MasqUtils.tolerance(current, prev, 0.01))  {
            if (current == value1) new Thread(action1).start();
            else if (current == value2) new Thread (action2).start();
        }
    }
}

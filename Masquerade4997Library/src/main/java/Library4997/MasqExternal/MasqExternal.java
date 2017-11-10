package Library4997.MasqExternal;

import java.math.BigDecimal;
import java.math.BigInteger;

/**
 * Created by Archish on 10/16/17.
 */

public class MasqExternal {

    public static final double NEVERREST_40_RPM = 160;
    public static final double NEVERREST_40_TICKS_PER_ROTATION = 1120;
    public static final double wheelDiameter = 4;
    public static final double gearRatio = .5;
    double CLICKS_PER_INCH = (Math.PI * wheelDiameter)/(NEVERREST_40_TICKS_PER_ROTATION * gearRatio);

    public static final int DEFAULT_SLEEP_TIME = 500;
    public static final double DEFAULT_TIMEOUT = 3;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = "AQL5v9v/////AAAAGey79Q2fZ0i7tLgjrpd85rZwqcK1HlVOI6UUmT02C7slX9+x5Qq" +
            "CfEwQhnuuB1hOh//uL2LnHYMViBgZtdjDGvmWvDvgKaonymopd0Y62ls2ZJfHhJ3fZYhF57Ce6ZepRI" +
            "FOumys4J4DssG83OT+DJUjUCG6ruZ88AYjxNzi+vhkTCxHVULQxLJCSQ7boG0t36RWIEmVwxXIHVI" +
            "3xbVeXwQL7vgm/0KmGW/KJFOuI2+wl1IDJdzDQHfavEA8FFkYTlnp/chHMbLu//BaqXprFHZ6OLh" +
            "OZoRWiFkg1N0zabreTxMNOYFP/rDNaYseXQVGGRSMHxF86kGs6LNHEO7qZZj/BU94zKpPMWyHYw29X477";

    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public static void sleep (double sleep) {
        try {Thread.sleep((long) sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public class KP {
        public static final double TURN = +0.025;
        public static final double DRIVE_ENCODER = +0.05;
        public static final double DRIVE_ANGULAR = +0.002;
        public static final double MOTOR_TELEOP = +0.003;
        public static final double MOTOR_AUTONOMOUS = +0.006;
    }
    public class KI {
        public static final double TURN = +0.002;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.000;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class KD {
        public static final double TURN = +0.0;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.00;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public class ID {
        public static final double TURN = +0.0;
        public static final double DRIVE = +0.0;
        public static final double MOTOR_TELEOP = +0.0;
        public static final double MOTOR_AUTONOMOUS = +0.00;
    }
    public static class VuMark {
        public static final boolean isCenter(String vuMark) {return vuMark.toLowerCase().contains("c");}
        public static final boolean isLeft(String vuMark) {return vuMark.toLowerCase().contains("l");}
        public static final boolean isRight(String vuMark) {return vuMark.toLowerCase().contains("g");}
        public static final boolean isUnKnown (String vuMark) {return vuMark.toLowerCase().contains("u");}
    }
}

package Library4997.MasqExternal;

/**
 * Created by Archish on 10/16/17.
 */

public class MasqExternal {

    public static final double MAX_RATE = 3100;
    public static final double TICKS_PER_ROTATION = 1120;
    public static final double wheelDiameter = 4;
    public static final double cmToInches = 2.54;
    public static final double gearRatio = 1;
    public static final double CLICKS_PER_CM = ((TICKS_PER_ROTATION / (wheelDiameter * cmToInches)) / Math.PI) / gearRatio;

    public static final int DEFAULT_SLEEP_TIME = 500;
    public static final double DEFAULT_TIMEOUT = 3;

    public static double ODS_WHITE = 0.7,
            ODS_BLACK = 0.3;
    public static String VUFORIA_KEY = "AQL5v9v/////AAAAGey79Q2fZ0i7tLgjrpd85rZwqcK1HlVOI6UUmT02C7slX9+x5Qq" +
            "CfEwQhnuuB1hOh//uL2LnHYMViBgZtdjDGvmWvDvgKaonymopd0Y62ls2ZJfHhJ3fZYhF57Ce6ZepRI" +
            "FOumys4J4DssG83OT+DJUjUCG6ruZ88AYjxNzi+vhkTCxHVULQxLJCSQ7boG0t36RWIEmVwxXIHVI" +
            "3xbVeXwQL7vgm/0KmGW/KJFOuI2+wl1IDJdzDQHfavEA8FFkYTlnp/chHMbLu//BaqXprFHZ6OLh" +
            "OZoRWiFkg1N0zabreTxMNOYFP/rDNaYseXQVGGRSMHxF86kGs6LNHEO7qZZj/BU94zKpPMWyHYw29X477";

    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public class KP {
        public static final double TURN = +0.005;
        public static final double DRIVE = +0.03;
        public static final double TELEOP = +0.1;
    }
    public class KI {
        public static final double TURN = +0.0002;
        public static final double DRIVE = +0.0;
        public static final double TELEOP = +0.0;
    }
    public class KD {
        public static final double TURN = +0.0;
        public static final double DRIVE = +0.0;
        public static final double TELEOP = +0.0;
    }
    public class ID {
        public static final double TURN = +0.0;
        public static final double DRIVE = +0.0;
        public static final double TELEOP = +0.0;
    }
}

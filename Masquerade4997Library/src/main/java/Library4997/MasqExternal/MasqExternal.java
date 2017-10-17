package Library4997.MasqExternal;

/**
 * Created by Archish on 10/16/17.
 */

public class MasqExternal {
    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
    public enum KP {
        TURN (+0.005),
        DRIVE (+0.03),
        TELEOP (+0.1);
        public final double value;
        KP (double value) {this.value = value;}
    }
    public enum KI {
        TURN (+0.0002),
        DRIVE (+0.0),
        TELEOP (+0.0);
        public final double value;
        KI (double value) {this.value = value;}
    }
    public enum KD {
        TURN (+0.0),
        DRIVE (+0.0),
        TELEOP (+0.0);
        public final double value;
        KD (double value) {this.value = value;}
    }
    public enum ID {
        TURN (+0.0),
        DRIVE (+0.0),
        TELEOP (+0.0);
        public final double value;
        ID (double value) {this.value = value;}
    }
}

package Library4997.MasqResources.MasqHelpers;

/**
 * Created by Archishmaan Peyyety on 2019-11-15.
 * Project: MasqLib
 */
public enum Strafe {
    FORWARD (0),
    BACKWARD (180),
    LEFT (-90),
    RIGHT (90);
    public final double value;
    Strafe (double value) {this.value = value;}
}

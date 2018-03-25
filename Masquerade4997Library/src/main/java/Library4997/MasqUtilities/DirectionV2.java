package Library4997.MasqUtilities;

/**
 * Created by Archish on 3/25/18.
 */

public enum  DirectionV2 {
    FORWARD (+1),
    BACKWARD (-1),
    LEFT (-1),
    RIGHT (+1);
    public final double value;
    DirectionV2 (double value) {this.value = value;}
}

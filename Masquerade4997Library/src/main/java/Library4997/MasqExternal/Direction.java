package Library4997.MasqExternal;

/**
 * This enum gives values to direction
 */

public enum Direction {
    //Number one is left, and two is right
    FORWARD (+1),
    BACKWARD (-1),
    LEFT (+1),
    RIGHT (-1);
    public final double value;
    Direction (double value) {this.value = value;}
}

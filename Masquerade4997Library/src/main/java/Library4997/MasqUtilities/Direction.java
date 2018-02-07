package Library4997.MasqUtilities;

/**
 * Gives a positive or negative multiplier associated to the needed motor spin direction of the associated direction.
 */

public enum Direction {
    FORWARD (+1),
    BACKWARD (-1),
    LEFT (+1),
    RIGHT (-1);
    public final double value;
    Direction (double value) {this.value = value;}
}

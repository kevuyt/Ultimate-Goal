package BasicLib4997.MasqRobot;

/**
 * This enum gives values to direction
 */

public enum Direction {
    FORWARD (+1.0),
    BACKWARD (-1.0),
    LEFT (+1.0),
    RIGHT (-1.0);
    public final double value;
    Direction (double value) {this.value = value;}
}

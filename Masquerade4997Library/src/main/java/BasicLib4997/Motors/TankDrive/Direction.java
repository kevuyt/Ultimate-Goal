package BasicLib4997.Motors.TankDrive;

/**
 * Created by Archish on 11/1/16.
 */

public enum Direction {

    /**
     *This enum gives values to direction
     */
    FORWARD (+1.0),
    BACKWARD (-1.0),
    LEFT (+1.0),
    RIGHT (-1.0);
    public final double value;
    Direction (double value) {this.value = value;}

}

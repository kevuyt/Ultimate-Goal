package Library4997.MasqExternal;

/**
 * This enum gives values to direction
 */

public enum Direction {
    //Number one is left, and two is right
    FORWARD (new double[]{1, 1}),
    BACKWARD (new double[]{-1, -1}),
    LEFT (new double[]{1, 1}),
    RIGHT (new double[]{-1, -1}),
    STRAFE_LEFT (new double[]{1, -1}),
    STRAFE_RIGHT(new double[]{-1, 1});
    public final double[] value;
    Direction (double value[]) {this.value = value;}
}

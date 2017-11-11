package Library4997.MasqExternal;

/**
 * Created by Archish on 11/11/17.
 */

public enum Strafe {
    LEFT (new double[]{-1, +1}),
    RIGHT (new double[]{+1, -1});
    public final double[] value;
    Strafe (double[] value) {this.value = value;}
}

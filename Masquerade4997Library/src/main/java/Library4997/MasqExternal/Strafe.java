package Library4997.MasqExternal;

/**
 * Position 0: leftFront
 * Position 1: rightFront
 * Position 2: rightBack
 * Position 3: leftBack
 */

public enum Strafe {
    LEFT (new int[] {-1 , 1, -1 , 1}),
    RIGHT (new int[] {1, -1, 1, -1});
    public final int[] value;
    Strafe (int[] value) {this.value = value;}
}

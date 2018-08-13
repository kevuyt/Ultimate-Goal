package Library4997.MasqResources;

/**
 * Created by Archishmaan Peyyety on 8/7/18.
 * Project: MasqLib
 */

public class MasqVector {
    private double x;
    private double y;
    public MasqVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public static MasqVector addVectors(MasqVector v1, MasqVector v2) {
        return new MasqVector(v1.getX() + v2.getX(), v1.getY() + v2.getY());
    }

}

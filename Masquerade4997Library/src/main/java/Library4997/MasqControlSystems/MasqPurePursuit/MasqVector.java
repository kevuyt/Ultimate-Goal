package Library4997.MasqControlSystems.MasqPurePursuit;

/**
 * Created by Archishmaan Peyyety on 8/13/18.
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

    public double getMagnitude () {
        return Math.hypot(getX(), getY());
    }

    public double getDirection () {
        return Math.atan2(getY(), getX());
    }

    public MasqPoint getTerminalPoint() {
        return new MasqPoint(getX(), getY());
    }
}

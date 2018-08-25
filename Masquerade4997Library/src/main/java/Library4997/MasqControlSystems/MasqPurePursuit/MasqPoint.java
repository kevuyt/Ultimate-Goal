package Library4997.MasqControlSystems.MasqPurePursuit;

/**
 * Created by Archishmaan Peyyety on 8/25/18.
 * Project: MasqLib
 */

public class MasqPoint {
    private double x;
    private double y;
    public MasqPoint(double x, double y) {
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

    public double distanceToPoint(MasqPoint point) {
        return Math.hypot(point.getX() - getX(), point.getY() - getY());
    }
}

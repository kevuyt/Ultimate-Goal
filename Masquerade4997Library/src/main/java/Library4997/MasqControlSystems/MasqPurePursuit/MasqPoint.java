package Library4997.MasqControlSystems.MasqPurePursuit;

/**
 * Created by Archishmaan Peyyety on 10/6/18.
 * Project: MasqLib
 */

public class MasqPoint {
    private double x, y;

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
    public boolean equals(MasqPoint point) {
        double x = Math.abs(getX());
        double y = Math.abs(getY());
        double x1 = Math.abs(point.getX());
        double y1 = Math.abs(point.getY());
        if ((x - x1) < 0.1 && (y - y1) < 0.1) return true;
        else return false;
    }
}

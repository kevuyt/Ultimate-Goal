package Library4997.MasqResources.MasqMath;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archishmaan Peyyety on 10/6/18.
 * Project: MasqLib
 */

public class MasqPoint implements MasqHardware {
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

    public MasqVector toVector() {
        return new MasqVector(getX(), getY());
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public String[] getDash() {
        return new String[]{
          "X: " + Double.toString(getX()),
                "Y: " + Double.toString(getY())
        };
    }
}

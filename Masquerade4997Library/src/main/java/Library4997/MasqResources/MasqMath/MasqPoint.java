package Library4997.MasqResources.MasqMath;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archishmaan Peyyety on 10/6/18.
 * Project: MasqLib
 */

public class MasqPoint implements MasqHardware {
    private double x, y, h;


    public MasqPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public MasqPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public double getH() {
        return h;
    }

    public void setH(double h) {
        this.h = h;
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

package Library4997.MasqControlSystems.MasqPurePursuit;

/**
 * Created by Archishmaan Peyyety on 2020-01-26.
 * Project: MasqLib
 */

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;
public class MasqWayPointLegacy implements MasqHardware {
    private double x, y, h, radius, velocity;


    public MasqWayPointLegacy(MasqPoint p) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = 0.1;
        this.velocity = 0;
    }
    public MasqWayPointLegacy(MasqPoint p, double radius) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = radius;
        velocity = 0;
    }
    public MasqWayPointLegacy(MasqPoint p, double radius, double velocity) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = radius;
        this.velocity = velocity;
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

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
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
package Library4997.MasqControlSystems.MasqPurePursuit;

import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqResources.MasqMath.MasqVector;

/**
 * Created by Archishmaan Peyyety on 2020-01-10.
 * Project: MasqLib
 */
public class MasqWayPoint implements MasqHardware {
    private double x, y, h, radius, minVelocity, lookAhead, angularCorrectionSpeed;


    public MasqWayPoint(MasqPoint p) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = 0.1;
        this.minVelocity = 0;
    }
    public MasqWayPoint(MasqPoint p, double radius) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = radius;
        minVelocity = 0;
    }
    public MasqWayPoint(MasqPoint p, double radius, double minVelocity) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = radius;
        this.minVelocity = minVelocity;
    }

    public MasqWayPoint(MasqPoint p, double radius, double minVelocity, double lookAhead, double angularCorrectionSpeed) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        this.radius = radius;
        this.minVelocity = minVelocity;
        this.lookAhead = lookAhead;
        this.angularCorrectionSpeed = angularCorrectionSpeed;
    }

    public double getLookAhead() {
        return lookAhead;
    }

    public void setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
    }

    public double getAngularCorrectionSpeed() {
        return angularCorrectionSpeed;
    }

    public void setAngularCorrectionSpeed(double angularCorrectionSpeed) {
        this.angularCorrectionSpeed = angularCorrectionSpeed;
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

    public double getMinVelocity() {
        return minVelocity;
    }

    public void setMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
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

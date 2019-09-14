package Library4997.MasqResources.MasqMath;

import Library4997.MasqResources.MasqHelpers.MasqHardware;

/**
 * Created by Archishmaan Peyyety on 8/13/18.
 * Project: MasqLib
 */

public class MasqVector implements MasqHardware {
    private double x;
    private double y;
    private String name;
    public MasqVector(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

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

    public MasqPoint toPoint() {
        return new MasqPoint(getX(), getY());
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public MasqVector unitVector () {
        return new MasqVector(getX()/getMagnitude(), getY()/getMagnitude());
    }

    public static MasqVector multiply(double scalar, MasqVector v) {
        return new MasqVector(v.getX() * scalar, v.getY() * scalar);
    }

    public double getMagnitude () {
        return Math.hypot(getX(), getY());
    }

    public double dotProduct(MasqVector v) {
        return (this.getX() * v.getX()) + (this.getY() * v.getY());
    }

    public double angleRad(MasqVector v) {
        return (Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
    }
    public double angleDeg(MasqVector v) {
        double deg = Math.toDegrees(Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
        if (Double.isNaN(deg)) return 0;
        return deg;
    }

    public double distanceToVector(MasqVector point) {
        return Math.hypot(point.getX() - getX(), point.getY() - getY());
    }

    public boolean equal(double radius, MasqVector v) {
        return distanceToVector(v) < radius;
    }

    public MasqVector displacement(MasqVector v) {
        return new MasqVector(v.getX() - getX(), v.getY() - getY());
    }

    public MasqVector projectOnTo(MasqVector v) {
        return multiply(dotProduct(v) / (v.getMagnitude() * v.getMagnitude()), v);
    }

    public double getDirection () {
        return Math.toDegrees(Math.atan(getY() / getX()));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + Double.toString(getX()),
                "Y: " + Double.toString(getY()),
                "Direction: " + Double.toString(getDirection())
        };
    }
}

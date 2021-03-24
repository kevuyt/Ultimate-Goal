package MasqLibrary.MasqMath;

import androidx.annotation.NonNull;

import static java.lang.Math.*;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqVector {
    private double x;
    private double y;
    private String name;

    public MasqVector(String name, double x, double y) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public double getX() {return x;}
    public double getY() {return y;}
    public void setX(double x) {this.x = x;}
    public void setY(double y) {this.y = y;}

    public double getMagnitude () {return hypot(getX(), getY());}
    public double getDirection () {return toDegrees(atan(getY() / getX()));}
    public MasqVector unitVector () {return new MasqVector(name, getX()/getMagnitude(), getY()/getMagnitude());}

    public MasqVector multiply(double scalar) {return new MasqVector(name, getX() * scalar, getY() * scalar);}
    public MasqVector displacement(MasqVector v) {return new MasqVector(name, v.getX() - getX(), v.getY() - getY());}
    public MasqVector projectOnTo(MasqVector v) {return v.unitVector().multiply(dotProduct(v.unitVector()));}
    public MasqVector add(MasqVector v) {return new MasqVector(name, getX() + v.getX(), getY() + v.getY());}
    public double dotProduct(MasqVector v) {return (this.getX() * v.getX()) + (this.getY() * v.getY());}
    public double angleTo(MasqVector v) {return toDegrees(atan2(getY(), getX()) - atan2(v.getY(), v.getX()));}
    public double distanceToVector(MasqVector v) {return hypot(v.getX() - getX(), v.getY() - getY());}
    public boolean equal(double radius, MasqVector v) {return distanceToVector(v) < radius;}

    public MasqVector setName(String name) {
        this.name = name;
        return this;
    }

    @NonNull
    @Override
    public String toString() {return String.format(US, "%s:\nX: %.2f\nY: %.2f", name, x, y);}
}

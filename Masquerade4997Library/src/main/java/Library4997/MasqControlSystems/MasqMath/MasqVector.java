package Library4997.MasqControlSystems.MasqMath;

/**
 * Created by Archishmaan Peyyety on 8/7/18.
 * Project: MasqLib
 */

public class MasqVector {
    double direction;
    double magnitude;
    public MasqVector(double direction, double magnitude) {
        this.direction = direction;
        this.magnitude = magnitude;
    }

    public double getDirection() {
        return direction;
    }

    public void setDirection(double direction) {
        this.direction = direction;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }

}

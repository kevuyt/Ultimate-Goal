package Library4997.MasqMotors;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
    private MasqMotor motor;
    double TPR = 0, wheelDiameter = 3, gearRatio = 1;
    int zeroPos = 0, currentPosition;
    private double CPR = (TPR / (wheelDiameter * Math.PI)) * gearRatio;
    public MasqEncoder (MasqMotor motor, double ppr) {
        this.motor = motor;
        this.TPR = ppr;
        resetEncoder();
    }
    public double getPosition () {
        currentPosition = (int) (motor.getCurrentPosition() - zeroPos);
        return currentPosition;
    }

    public double getInches () {
        return getPosition() / CPR;
    }

    private double position() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        zeroPos = (int) position();
        currentPosition = 0;
    }

    public double getClicksPerInch() {
        return CPR;
    }

    public double getTPR() {
        return TPR;
    }

    public void setPPR(double PPR) {
        this.TPR = PPR;
    }

    public MasqMotor getMotor() {
        return motor;
    }

    public void setMotor(MasqMotor motor) {
        this.motor = motor;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public void setWheelDiameter(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(int gearRatio) {
        this.gearRatio = gearRatio;
    }
}

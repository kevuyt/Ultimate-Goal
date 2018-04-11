package Library4997.MasqMotors;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
    private MasqMotor motor;
    int PPR = 1440, wheelDiameter = 3, gearRatio = 1;
    int zeroPos = 0, currentPosition;
    private double CPR = (PPR / (wheelDiameter * Math.PI)) * gearRatio;
    public MasqEncoder (MasqMotor motor) {
        this.motor = motor;
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

    public int getPPR() {
        return PPR;
    }

    public void setPPR(int CPR) {
        this.PPR = CPR;
    }

    public MasqMotor getMotor() {
        return motor;
    }

    public void setMotor(MasqMotor motor) {
        this.motor = motor;
    }

    public int getWheelDiameter() {
        return wheelDiameter;
    }

    public void setWheelDiameter(int wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }

    public int getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(int gearRatio) {
        this.gearRatio = gearRatio;
    }
}

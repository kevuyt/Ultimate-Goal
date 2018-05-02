package Library4997.MasqSensors;

import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqUtilities.MasqUtils;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
    private MasqMotor motor;
    double TPR = 0, wheelDiameter = 3, gearRatio = 1;
    int zeroPos = 0, currentPosition;
    private double CPI = (TPR / (wheelDiameter * Math.PI)) * gearRatio;
    public enum Encoder {
        ANDYMARK_STANDARD(MasqUtils.NEVERREST_ORBITAL_20_TICKS_PER_ROTATION),
        ANDYMARK_317(1.0),
        US_DIGITAL(MasqUtils.US_ERT_ENCODER_TICKS_PER_ROTATION);
        public final double encoder;
        Encoder (double encoder){this.encoder = encoder;}
    }
    public MasqEncoder (MasqMotor motor, Encoder type) {
        this.motor = motor;
        this.TPR = type.encoder;
        resetEncoder();
    }
    public double getPosition () {
        currentPosition = (int) (motor.getCurrentPosition() - zeroPos);
        return currentPosition;
    }

    public double getInches () {
        return getPosition() / CPI;
    }

    private double position() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        zeroPos = (int) position();
        currentPosition = 0;
    }

    public double getClicksPerInch() {
        return CPI;
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

    public void setEncoderType (Encoder encoderType) {

    }
}

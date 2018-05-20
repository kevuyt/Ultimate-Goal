package Library4997.MasqSensors;

import Library4997.MasqMotors.MasqMotor;

/**
 * Created by Archish on 3/14/18.
 */

public class MasqEncoder {
    private MasqMotorModel model;
    private MasqMotor motor;
    private double wheelDiameter = 4, gearRatio = 1;
    private double currentPosition, zeroPos;
    private double clicksPerInch = (model.CPR / (wheelDiameter * Math.PI)) * gearRatio;
    public enum MasqMotorModel {
        NEVEREST20, NEVEREST40, NEVEREST60, US_DIGITAL;
        public double CPR;
        public static double DEFAULT_CPR = 0;
        public static double CPR(MasqMotorModel motorModel) {
            switch (motorModel){
                case NEVEREST20:
                    return 537.6;
                case NEVEREST40:
                    return 1120;
                case NEVEREST60:
                    return 1680;
                case US_DIGITAL:
                    return 1440;
            }
            return DEFAULT_CPR;
        }
        public int RPM;
        public static int DEFAULT_RPM = 0;
        public static int RPM(MasqMotorModel motorModel) {
            switch (motorModel) {
                case NEVEREST20:
                    return 340;
                case NEVEREST40:
                    return 160;
                case NEVEREST60:
                    return 105;
            }
            return DEFAULT_RPM;
        }
    }
    public MasqEncoder(MasqMotor motor, MasqMotorModel model) {
        this.model = model;
        this.motor = motor;
    }

    public double getRelativePosition() {
        currentPosition = (int) (motor.getCurrentPosition() - zeroPos);
        return currentPosition;
    }

    public double getInches () {
        return getRelativePosition() / clicksPerInch;
    }

    public double getAbsolutePosition() {
        return motor.getCurrentPosition();
    }

    public void resetEncoder() {
        zeroPos = (int) getAbsolutePosition();
        currentPosition = 0;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public void setWheelDiameter(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
    }

    public double getRPM () {
        return model.RPM;
    }
    public double getClicksPerRotation () {
        return model.CPR;
    }
    public double getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    public void setModel(MasqMotorModel model) {
        this.model = model;
    }
}

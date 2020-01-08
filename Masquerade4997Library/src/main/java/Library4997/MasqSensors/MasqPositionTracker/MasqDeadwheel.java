package Library4997.MasqSensors.MasqPositionTracker;

import Library4997.MasqMotors.MasqMotor;

/**
 * Created by Archishmaan Peyyety on 2020-01-08.
 * Project: MasqLib
 */
public class MasqDeadwheel {
    private MasqMotor motor;
    private WheelPosition wheelPosition;
    private Measurement measurement;
    private double latestPosition, prevPosition;
    public enum WheelPosition {
        TOP, BOTTOM, LEFT, RIGHT, CENTER
    }
    public enum Measurement {
        X, Y
    }

    public MasqDeadwheel(MasqMotor motor, WheelPosition wheelPosition, Measurement measurement) {
        this.motor = motor;
        this.wheelPosition = wheelPosition;
        this.measurement = measurement;
    }

    public void reset(){
        motor.resetEncoder();
    }
    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public WheelPosition getWheelPosition() {
        return wheelPosition;
    }

    public Measurement getMeasurement() {
        return measurement;
    }

    public double getChange() {
        latestPosition = getPosition();
        double change = latestPosition - prevPosition;
        prevPosition = latestPosition;
        return change;
    }
}

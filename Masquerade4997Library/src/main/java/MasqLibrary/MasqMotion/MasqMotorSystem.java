package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import MasqLibrary.MasqSensors.MasqTouchSensor;

import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqMotorSystem {
    MasqMotor[] motors;
    String name;

    public MasqMotorSystem(String name, MasqMotor... motors) {
        this.motors = motors;
        this.name = name;
    }

    public void resetEncoders() {for (MasqMotor masqMotor : motors) masqMotor.resetEncoder();}

    public double getInches () {
        double sum = 0;
        for (MasqMotor masqMotor : motors) sum += masqMotor.getInches();
        return sum / motors.length;
    }
    public double getCurrentPosition() {
        double sum = 0;
        for(MasqMotor motor : motors) sum += motor.getCurrentPosition();
        return sum / motors.length;
    }

    public void setPower(double power) {
        for (MasqMotor masqMotor : motors) masqMotor.setPower(power);
    }
    public void setLimits (MasqTouchSensor min, MasqTouchSensor max) {
        for (MasqMotor masqMotor : motors) masqMotor.setLimits(min, max);
    }
    public void setVelocityControl (boolean velocityControl) {
        for (MasqMotor masqMotor : motors) masqMotor.setVelocityControl(velocityControl);
    }

    public double getPower() {
        double sum = 0;
        for (MasqMotor masqMotor: motors) sum += Math.abs(masqMotor.getPower());
        return sum/motors.length;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nNum Motors: %d\nInches: %f", name, motors.length, getInches());
    }
}
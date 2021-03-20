package MasqueradeLibrary.MasqMotion;

import androidx.annotation.NonNull;

import static MasqueradeLibrary.MasqResources.MasqUtils.*;
import static java.lang.Math.*;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqDriveTrain {
    public MasqMotorSystem leftDrive, rightDrive;
    public MasqMotor motor1, motor2, motor3, motor4;

    public MasqDriveTrain() {
        motor1 = new MasqMotor("LeftFront");
        motor2 = new MasqMotor("LeftBack");
        motor3 = new MasqMotor("RightFront");
        motor4 = new MasqMotor("RightBack");

        leftDrive = new MasqMotorSystem("LeftDrive", motor1, motor2);
        rightDrive = new MasqMotorSystem("RightDrive", motor3, motor4);

    }

    public void resetEncoders () {
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }

    public void setPower(double leftPower, double rightPower) {
        rightDrive.setPower(rightPower);
        leftDrive.setPower(leftPower);
    }
    public void setPower(double power){
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public double getInches() {return (leftDrive.getInches() + rightDrive.getInches())/2;}

    public void setVelocityControl(boolean velocityControl) {
        leftDrive.setVelocityControl(velocityControl);
        rightDrive.setVelocityControl(velocityControl);
    }

    public void setPowerMECH(double angle, double speed, double turnPower) {
        angle = toRadians(angle);
        double adjustedAngle = angle + PI/4;

        double leftFront = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * DEFAULT_TURN_MULTIPLIER;
        double leftBack = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * DEFAULT_TURN_MULTIPLIER;
        double rightFront = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * DEFAULT_TURN_MULTIPLIER;
        double rightBack = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * DEFAULT_TURN_MULTIPLIER;

        double max = max(abs(leftFront), abs(leftBack), abs(rightFront), abs(rightBack));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        motor1.setPower(leftFront);
        motor2.setPower(leftBack);
        motor3.setPower(rightFront);
        motor4.setPower(rightBack);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "DriveTrain:\nInches Traveled: %.2f", getInches());
    }
}
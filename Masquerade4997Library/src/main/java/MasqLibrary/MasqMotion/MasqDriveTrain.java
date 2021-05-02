package MasqLibrary.MasqMotion;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import static MasqLibrary.MasqResources.MasqUtils.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static java.lang.Math.*;
import static java.util.Locale.US;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqDriveTrain {
    public MasqMotorSystem leftDrive, rightDrive;
    public MasqMotor motor1, motor2, motor3, motor4;

    public MasqDriveTrain(Direction direction) {
        motor1 = new MasqMotor("leftFront", direction.inverted());
        motor2 = new MasqMotor("leftBack", direction.inverted());
        motor3 = new MasqMotor("rightFront", direction);
        motor4 = new MasqMotor("rightBack", direction);

        leftDrive = new MasqMotorSystem("LeftDrive", motor1, motor2);
        rightDrive = new MasqMotorSystem("RightDrive", motor3, motor4);
    }
    public MasqDriveTrain() {this(FORWARD);}

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
        angle += PI / 4;
        double speedMultiplier = speed * sqrt(2);

        double leftFront = (sin(angle) * speedMultiplier) + turnPower;
        double leftBack = (cos(angle) * speedMultiplier) + turnPower;
        double rightFront = (cos(angle) * speedMultiplier) - turnPower;
        double rightBack = (sin(angle) * speedMultiplier) - turnPower;

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
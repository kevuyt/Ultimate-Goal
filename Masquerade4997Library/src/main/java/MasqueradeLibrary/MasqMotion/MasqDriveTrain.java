package MasqueradeLibrary.MasqMotion;

import com.qualcomm.robotcore.hardware.HardwareMap;

import static MasqueradeLibrary.MasqMotion.MasqMotor.MasqMotorModel;
import static MasqueradeLibrary.MasqMotion.MasqMotor.MasqMotorModel.REVHDHEX20;
import static MasqueradeLibrary.MasqResources.MasqUtils.*;
import static java.lang.Math.*;

/**
 * Created by Keval Kataria on 3/15/2021
 */

public class MasqDriveTrain {
    public MasqMotorSystem leftDrive, rightDrive;
    public MasqMotor motor1, motor2, motor3, motor4;
    public MasqMotorModel model;

    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MasqMotorModel model) {
        motor1 = new MasqMotor(name1, model, hardwareMap);
        motor2 = new MasqMotor(name2, model, hardwareMap);
        motor3 = new MasqMotor(name3, model, hardwareMap);
        motor4 = new MasqMotor(name4, model, hardwareMap);

        leftDrive = new MasqMotorSystem("LeftDrive", motor1, motor2);
        rightDrive = new MasqMotorSystem("RightDrive", motor3, motor4);

        this.model = model;
    }
    public MasqDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        new MasqDriveTrain(name1, name2, name3, name4, hardwareMap, REVHDHEX20);
    }

    public MasqDriveTrain(HardwareMap hardwareMap, MasqMotorModel motorModel){
        new MasqDriveTrain("LeftFront", "LeftBack", "RightFront", "RightBack", hardwareMap);
    }
    public MasqDriveTrain(HardwareMap hardwareMap) {
        new MasqDriveTrain(hardwareMap, REVHDHEX20);
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
    public void setPower(double leftFront, double leftBack, double rightFront, double rightBack) {
        motor1.setPower(leftFront);
        motor2.setPower(leftBack);
        motor3.setPower(rightFront);
        motor4.setPower(rightBack);
    }

    public double getInches() {return (leftDrive.getInches() + rightDrive.getInches())/2;}

    public double getCurrentPosition() {return getInches() * model.CPR;}

    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }

    public void setVelocityControl(boolean velocityControl) {
        leftDrive.setVelocityControl(velocityControl);
        rightDrive.setVelocityControl(velocityControl);
    }

    public void setPowerMECH(double angle, double speed, double turnPower) {
        angle = toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;

        double leftFront = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * DEFAULT_TURN_MULTIPLIER;
        double leftBack = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * DEFAULT_TURN_MULTIPLIER;
        double rightFront = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * DEFAULT_TURN_MULTIPLIER;
        double rightBack = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * DEFAULT_TURN_MULTIPLIER;

        double max = max(max(abs(leftFront), abs(leftBack)), max(abs(rightFront), abs(rightBack)));
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
}
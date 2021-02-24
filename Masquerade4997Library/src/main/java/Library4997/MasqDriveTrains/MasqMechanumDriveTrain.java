package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.*;
import Library4997.MasqResources.MasqHardware;
import Library4997.MasqUtils;

import static Library4997.MasqUtils.*;
import static java.lang.Math.*;

public class MasqMechanumDriveTrain extends MasqDriveTrain implements MasqHardware {

    public MasqMechanumDriveTrain(HardwareMap hardwareMap){
        super(hardwareMap);
    }
    public MasqMechanumDriveTrain(HardwareMap hardwareMap, MasqMotorModel motorModel){
        super(hardwareMap, motorModel);
    }
    public MasqMechanumDriveTrain(MasqMotorSystem left, MasqMotorSystem right) {
        super(left, right);
    }

    public void setVelocityMECH(double angle, double speed, double targetHeading) {
        double turnPower = angleController.getOutput(adjustAngle(targetHeading - getTracker().getHeading()));
        angle = toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double leftBack = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double max = max(max(abs(leftFront), abs(leftBack)), max(abs(rightFront), abs(rightBack)));

        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }

    public void setVelocityMECH(double angle, double speed) {
        setVelocityMECH(angle, speed, getTracker().getHeading());
    }
}
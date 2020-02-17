package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqPositionTracker;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.DashBoard;


public class MasqMechanumDriveTrain extends MasqDriveTrain implements MasqHardware {
    public static MasqPIDController angleCorrectionController = new MasqPIDController(0.05);
    private MasqPositionTracker tracker;

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
        double turnPower = angleCorrectionController.getOutput(MasqUtils.adjustAngle(targetHeading - tracker.getHeading()));
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower  * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        DashBoard.getDash().create("Left Front Power: ", leftFront);
        DashBoard.getDash().create("Left Back Power: ", leftBack);
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }


    public void setVelocityMECH(double angle, double speed) {
        setVelocityMECH(angle, speed, tracker.getHeading());
    }
    public void setVelocityMECH(double angle, double speed, double targetHeading, double turnAdjustment) {
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        leftFront -= turnAdjustment;
        leftBack -= turnAdjustment;
        rightBack += turnAdjustment;
        rightBack += turnAdjustment;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
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

    public void setPowerMECH(double angle, double speed, double targetHeading, double turnAdjustment) {
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        leftFront -= turnAdjustment;
        leftBack -= turnAdjustment;
        rightBack += turnAdjustment;
        rightBack += turnAdjustment;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setPower(leftFront);
        leftDrive.motor2.setPower(leftBack);
        rightDrive.motor1.setPower(rightFront);
        rightDrive.motor2.setPower(rightBack);
    }

    public void setTurnControllerKp (double kp) {
        angleCorrectionController.setKp(kp);
    }

    public void setPowerMECH(double angle, double speed, double targetHeading) {
        double turnPower = angleCorrectionController.getOutput(tracker.getHeading() - targetHeading);
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower  * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setPower(leftFront);
        leftDrive.motor2.setPower(leftBack);
        rightDrive.motor1.setPower(rightFront);
        rightDrive.motor2.setPower(rightBack);
    }

    public MasqPositionTracker getTracker() {
        return tracker;
    }

    public void setTracker(MasqPositionTracker tracker) {
        this.tracker = tracker;
    }
}

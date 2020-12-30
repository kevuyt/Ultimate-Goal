package Library4997.MasqDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqResources.MasqMath.MasqPIDController;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker;
import Library4997.MasqResources.MasqHelpers.MasqHardware;
import Library4997.MasqResources.MasqMath.MasqVector;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqWrappers.DashBoard;

import static Library4997.MasqResources.MasqUtils.xSpeedController;
import static Library4997.MasqResources.MasqUtils.ySpeedController;


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
        double leftBack = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MasqUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
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
        DashBoard.getDash().create("LeftFront Power: ",leftFront);
        DashBoard.getDash().create("Turn Power: ",turnPower);
        DashBoard.getDash().create("Speed",speed);
        DashBoard.getDash().create("Angle", angle);

    }

    public void setVelocityMECHXY(double angle, MasqVector current, MasqVector target, double targetHeading) {
        double turnPower = angleCorrectionController.getOutput(MasqUtils.adjustAngle(targetHeading - tracker.getHeading()));
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;

        double speedx = xSpeedController.getOutput(target.getX() - current.getX());
        double leftFrontX = (Math.sin(adjustedAngle) * speedx * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBackX = (Math.cos(adjustedAngle) * speedx * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFrontX = (Math.cos(adjustedAngle) * speedx * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBackX = (Math.sin(adjustedAngle) * speedx * MasqUtils.DEFAULT_SPEED_MULTIPLIER);

        double speedy = ySpeedController.getOutput(target.getY() - current.getY());
        double leftFrontY = (Math.sin(adjustedAngle) * speedy * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBackY = (Math.cos(adjustedAngle) * speedy * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFrontY = (Math.cos(adjustedAngle) * speedy * MasqUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBackY = (Math.sin(adjustedAngle) * speedy * MasqUtils.DEFAULT_SPEED_MULTIPLIER);


        double maxX = Math.max(Math.max(Math.abs(leftFrontX), Math.abs(leftBackX)), Math.max(Math.abs(rightFrontX), Math.abs(rightBackX)));
        double maxY = Math.max(Math.max(Math.abs(leftFrontY), Math.abs(leftBackY)), Math.max(Math.abs(rightFrontY), Math.abs(rightBackY)));

        if (maxX > 1) {
            leftFrontX /= maxX;
            leftBackX /= maxX;
            rightFrontX /= maxX;
            rightBackX /= maxX;
        }

        if (maxY > 1) {
            leftFrontY /= maxY;
            leftBackY /= maxY;
            rightFrontY /= maxY;
            rightBackY /= maxY;
        }

        double powerAdjustment = turnPower * MasqUtils.DEFAULT_TURN_MULTIPLIER;
        leftDrive.motor1.setVelocity(leftFrontX + leftFrontY - powerAdjustment);
        leftDrive.motor2.setVelocity(leftBackX + leftBackY - powerAdjustment);
        rightDrive.motor1.setVelocity(rightFrontX + rightFrontY + powerAdjustment);
        rightDrive.motor2.setVelocity(rightBackX + rightBackY + powerAdjustment);
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

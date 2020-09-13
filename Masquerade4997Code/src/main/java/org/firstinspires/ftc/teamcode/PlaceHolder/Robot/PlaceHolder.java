package org.firstinspires.ftc.teamcode.PlaceHolder.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.RingDetector;
import org.openftc.easyopencv.OpenCvCameraRotation;

import Library4997.MasqCV.MasqCamera;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqPositionTracker;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;
import Library4997.MasqWrappers.MasqController;

import static Library4997.MasqCV.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqSensors.MasqClock.Resolution.SECONDS;


/**
 * Created by Keval Kataria on 9/12/2020
 */
public class PlaceHolder extends MasqRobot {

    public MasqCamera camera;
    public MasqMotor intake, transfer, lift, shooter;
    public MasqServo claw;
    private boolean prevStateClaw =false, taskStateClaw =false;


    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, hardwareMap);
        intake = new MasqMotor("intake", MasqMotorModel.REVTHROUGHBORE, hardwareMap);
        transfer = new MasqMotor("transfer", MasqMotorModel.REVTHROUGHBORE, hardwareMap);
        shooter = new MasqMotor("shooter", MasqMotorModel.REVTHROUGHBORE, hardwareMap);
        claw = new MasqServo("claw",hardwareMap);
        tracker = new MasqPositionTracker(intake, transfer, shooter, hardwareMap);
        dash = DashBoard.getDash();
    }

    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        tracker.setPosition(MasqPositionTracker.DeadWheelPosition.THREE);
        tracker.setXRadius(5.68);
        tracker.setTrackWidth(14.625);

        driveTrain.setTracker(tracker);

        MasqUtils.driveController = new MasqPIDController(0.005);
        MasqUtils.angleController = new MasqPIDController(0.003);
        MasqUtils.turnController = new MasqPIDController(0.003);
        MasqUtils.velocityTeleController = new MasqPIDController(0.001);
        MasqUtils.velocityAutoController = new MasqPIDController(0.001);
        MasqUtils.xySpeedController = new MasqPIDController(0.08, 0, 0);
        MasqUtils.xyAngleController = new MasqPIDController(0.06, 0, 0);

        intake.setWheelDiameter(2);
        lift.setClosedLoop(true);
        lift.encoder.setWheelDiameter(2);
        lift.setKp(0.01);
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();

        scaleServos();
        resetServos();
    }

    public void initCamera(HardwareMap hardwareMap) {
        RingDetector detector = new RingDetector();
        detector.setClippingMargins(90,90,110,50);
        camera = new MasqCamera(detector, WEBCAM, hardwareMap);
        camera.start(OpenCvCameraRotation.UPSIDE_DOWN);
    }

    private void scaleServos() {
        claw.scaleRange(0,1);
    }

    private void resetServos() {
        claw.setPosition(0);
    }

    public void toggleClaw(MasqController controller) {

        boolean currStateClaw = false;
        if (controller.y()) {
            currStateClaw = true;
        } else {
            if (prevStateClaw) {
                taskStateClaw = !taskStateClaw;
            }
        }

        prevStateClaw = currStateClaw;

        if (taskStateClaw) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
    }

    public void stop(double time) {
        MasqClock clock = new MasqClock();
        while(!clock.elapsedTime(time, SECONDS) && opModeIsActive()) {
            driveTrain.setVelocity(0);
        }
        driveTrain.setPower(0);
    }
    public void shoot() {

    }
}
package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneFoundationHook;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;
import MasqCV.detectors.skystone.SkystoneDetector;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class MarkOne extends MasqRobot {

    public MasqServo blockGrabber, blockRotater, blockPusher, capper, blockStopper;
    public MarkOneFoundationHook foundationHook;
    public MasqMotor  lift;
    public MasqMotorSystem intake;
    private OpenCvWebcam webcam;
    public SkystoneDetector detector;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE,MasqMotorModel.REVHDHEX40, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);
        capper = new MasqServo("capper", hardwareMap);
        blockStopper = new MasqServo("blockStopper", hardwareMap);
        tracker = new MasqPositionTracker(lift, intake.motor1, hardwareMap);
        foundationHook = new MarkOneFoundationHook(hardwareMap);
        dash = DashBoard.getDash();
    }

    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        MasqUtils.driveController = new MasqPIDController(0.005,0,0);
        MasqUtils.angleController = new MasqPIDController(0.01,0,0);
        MasqUtils.turnController = new MasqPIDController(0.015,0,0);
        MasqUtils.velocityTeleController = new MasqPIDController(0.002, 0, 0);
        MasqUtils.velocityAutoController = new MasqPIDController(0.002, 0, 0);
        lift.encoder.setWheelDiameter(1);
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        lift.setClosedLoop(true);
        lift.setKp(0.001);
        scaleServos();
        resetServos();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detector = new SkystoneDetector();
        webcam.setPipeline(detector);
        startCV();
        detector.setClippingMargins(100,80,110,70);
    }

    public void startCV(){
        webcam.openCameraDevice();
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
    }

    public void stopCV(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }

    private void scaleServos() {
        blockPusher.scaleRange(0, 0.5);
        blockGrabber.scaleRange(0, 0.5);
        blockRotater.scaleRange(0.02, 0.7);
        capper.scaleRange(0.5,1);
        blockStopper.scaleRange(0.25,1);
    }

    private void resetServos() {
        blockPusher.setPosition(0);
        blockRotater.setPosition(0);
        blockGrabber.setPosition(1);
        foundationHook.lower();
        capper.setPosition(0);
        blockStopper.setPosition(0);
    }
}
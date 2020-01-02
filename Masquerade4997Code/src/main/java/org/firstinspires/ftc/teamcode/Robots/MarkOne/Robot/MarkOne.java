package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneFoundationHook;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneSideGrabber;

import Library4997.MasqCV.MasqCV;
import Library4997.MasqCV.detectors.skystone.SkystoneDetector;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqPositionTracker;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;


/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class MarkOne extends MasqRobot {

    public MasqServo blockGrabber, blockRotater, blockPusher, capper;
    public MarkOneFoundationHook foundationHook;
    public MarkOneSideGrabber sideGrabber;
    public MasqMotor lift;
    public MasqMotorSystem intake;
    public MasqCV cv;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE,MasqMotorModel.USDIGITAL_E4T, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);
        capper = new MasqServo("capper", hardwareMap);
        sideGrabber = new MarkOneSideGrabber(hardwareMap);
        tracker = new MasqPositionTracker(intake.motor2, intake.motor1, hardwareMap) {
            @Override
            public double getXPosition() {
                return intake.motor1.encoder.getInches();
            }
            @Override
            public double getYPosition() {
                return intake.motor2.encoder.getInches();
            }
        };
        foundationHook = new MarkOneFoundationHook(hardwareMap);
        dash = DashBoard.getDash();
    }

    public void init(HardwareMap hardwareMap) throws InterruptedException {
        mapHardware(hardwareMap);
        driveTrain.setTracker(tracker);
        setPosition(MasqPositionTracker.DeadWheelPosition.BOTH_PERPENDICULAR);
        tracker.setXRadius(5.68 * 1.91667);
        tracker.setYRadius(7.11 * 1.91667);
        MasqUtils.driveController = new MasqPIDController(0.005);
        MasqUtils.angleController = new MasqPIDController(0.01);
        MasqUtils.turnController = new MasqPIDController(0.015);
        MasqUtils.velocityTeleController = new MasqPIDController(0.002);
        MasqUtils.velocityAutoController = new MasqPIDController(0.002);
        lift.encoder.setWheelDiameter(1);
        intake.motor1.encoder.setWheelDiameter(2);
        intake.motor2.encoder.setWheelDiameter(2);
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        lift.setClosedLoop(true);
        lift.setKp(0.001);
        scaleServos();
        resetServos();
        SkystoneDetector detector = new SkystoneDetector();
        detector.setClippingMargins(100,80,110,70);
        cv = new MasqCV(detector, MasqCV.Cam.WEBCAM, hardwareMap);
    }

    private void scaleServos() {
        blockPusher.scaleRange(0, 0.5);
        blockGrabber.scaleRange(0, 0.5);
        blockRotater.scaleRange(0.02, 0.7);
        capper.scaleRange(0.5,1);
    }

    private void resetServos() throws InterruptedException {
        blockPusher.setPosition(0);
        blockRotater.setPosition(0);
        blockGrabber.setPosition(1);
        foundationHook.raise();
        sideGrabber.reset();
        capper.setPosition(0);
    }
}
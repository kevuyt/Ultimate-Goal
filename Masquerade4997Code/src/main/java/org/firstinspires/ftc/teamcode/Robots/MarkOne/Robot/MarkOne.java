package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneFoundationHook;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneSideGrabber;

import Library4997.MasqCV.MasqCV;
import Library4997.MasqCV.detectors.skystone.SkystoneDetector;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqWayPoint;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqPositionTracker;
import Library4997.MasqPositionTrackerV2;
import Library4997.MasqResources.MasqMath.MasqPoint;
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
    public MasqMotor lift, tapeMeasure;
    public MasqMotorSystem intake;
    public MasqCV cv;
    public MasqPositionTrackerV2 trackerV2;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.NEVEREST60, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE, MasqMotorModel.REVTHROUGHBORE, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);
        capper = new MasqServo("capper", hardwareMap);
        sideGrabber = new MarkOneSideGrabber(hardwareMap);
        tapeMeasure = new MasqMotor("X", MasqMotorModel.USDIGITAL_E4T, DcMotorSimple.Direction.REVERSE,hardwareMap);
        tracker = new MasqPositionTracker(tapeMeasure,intake.motor1, intake.motor2, hardwareMap);
        trackerV2 = new MasqPositionTrackerV2(tapeMeasure,intake.motor1, intake.motor2, hardwareMap);
        foundationHook = new MarkOneFoundationHook(hardwareMap);
        dash = DashBoard.getDash();
    }

    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        tracker.setPosition(MasqPositionTracker.DeadWheelPosition.THREE);
        tracker.setXRadius(5.68);
        tracker.setTrackWidth(14.625);

        trackerV2.setXRadius(5.68);
        trackerV2.setTrackWidth(14.625);

        driveTrain.setTracker(tracker);
        MasqUtils.driveController = new MasqPIDController(0.005);
        MasqUtils.angleController = new MasqPIDController(0.003);
        MasqUtils.turnController = new MasqPIDController(0.015);
        MasqUtils.velocityTeleController = new MasqPIDController(0.001);
        MasqUtils.velocityAutoController = new MasqPIDController(0.004);
        MasqUtils.xySpeedController = new MasqPIDController(0.04, 0, 0);
        MasqUtils.xyAngleController = new MasqPIDController(0.05, 0, 0);
        lift.encoder.setWheelDiameter(1);
        tapeMeasure.setWheelDiameter(2);
        intake.setWheelDiameter(2);
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        lift.setClosedLoop(true);
        lift.setKp(0.007);
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
        sideGrabber.scaleServos();
    }

    public void resetServos() {
        blockPusher.setPosition(0);
        blockRotater.setPosition(0);
        blockGrabber.setPosition(1);
        foundationHook.raise();
        sideGrabber.reset();
        capper.setPosition(0);
    }

    public MasqWayPoint getCurrentWayPoint() {
        return new MasqWayPoint(new MasqPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading()), 10, 1);
    }
}
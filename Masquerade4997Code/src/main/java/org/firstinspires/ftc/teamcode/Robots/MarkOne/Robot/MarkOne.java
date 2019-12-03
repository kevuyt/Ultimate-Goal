package org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.SubSystems.MarkOneFoundationHook;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqHelpers.Strafe;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class MarkOne extends MasqRobot {

    public MasqServo blockGrabber, blockRotater, blockPusher, capper, blockStopper;
    public MarkOneFoundationHook foundationHook;
    public MasqMotor  lift;
    public MasqMotorSystem intake;
    public MarkOneDetector detector;

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
        tracker = new MasqPositionTracker(lift, intake.motor1, hardwareMap); //Replace motors when odometry is incorporating
        foundationHook = new MarkOneFoundationHook(hardwareMap);
        dash = DashBoard.getDash();
        detector = new MarkOneDetector(hardwareMap);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        scaleServos();
        resetServos();
        lift.encoder.setWheelDiameter(1);
        MasqUtils.driveController = new MasqPIDController(0.005,0,0);
        MasqUtils.angleController = new MasqPIDController(0.01,0,0);
        MasqUtils.turnController = new MasqPIDController(0.015,0,0);
        MasqUtils.velocityTeleController = new MasqPIDController(0.002, 0, 0);
        MasqUtils.velocityAutoController = new MasqPIDController(0.002, 0, 0);
        driveTrain.setClosedLoop(true);
        lift.setClosedLoop(true);
        lift.setKp(0.001);
        driveTrain.resetEncoders();
        detector.start();
        detector.setClippingMargins(100,80,110,70);
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

    public void runStoneLeft() {
        strafe(Math.hypot(15,18),Math.toDegrees(Math.atan2(-18,15)));
        intake.setVelocity(1);
        drive(15);
        sleep(1);
        intake.setVelocity(0);
        blockPusher.setPosition(0);
        blockGrabber.setPosition(0);
        dash.create(1);
        dash.update();
        drive(18, Direction.BACKWARD);
        driveTrain.stopDriving();
        dash.create(2);
        dash.update();
        sleep(10);
        dash.create(3);
        dash.update();
        strafe(90, Strafe.LEFT,500);
        blockPusher.setPosition(1);
        drive(5);
        foundationHook.lower();
        sleep(1);
        drive(30,Direction.BACKWARD);
        foundationHook.raise();
        sleep(1);
        strafe(30,Strafe.RIGHT);
        foundationHook.mid();
        drive(15);
        turnAbsolute(90);
        lift.runToPosition(15,0.5);
        blockRotater.setPosition(1);
        sleep();
        lift.runToPosition(0,0.5);
        blockGrabber.setPosition(1);
        sleep();
        lift.runToPosition(15,0.5);
        blockRotater.setPosition(0);
        sleep();
        lift.runToPosition(0,0.5);
        strafe(20,Strafe.RIGHT);
        turnAbsolute(0);
        strafe(84,Strafe.RIGHT);
        intake.setVelocity(1);
        drive(5);
        sleep(15);
        intake.setVelocity(0);
        blockPusher.setPosition(0);
        blockGrabber.setPosition(0);
        drive(5,Direction.BACKWARD);
        strafe(84,Strafe.LEFT);
        blockPusher.setPosition(1);
        turnAbsolute(90);
        strafe(20,Strafe.LEFT);
        lift.runToPosition(15,0.5);
        blockRotater.setPosition(1);
        sleep();
        lift.runToPosition(0,0.5);
        blockGrabber.setPosition(1);
        sleep();
        lift.runToPosition(15,0.5);
        blockRotater.setPosition(0);
        sleep();
        lift.runToPosition(0,0.5);
        strafe(20,Strafe.RIGHT);
        drive(15);
    }
    public void runStoneMiddle() {}
    public void runStoneRight() {}
}
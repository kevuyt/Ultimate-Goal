package org.firstinspires.ftc.teamcode.PlaceHolder.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.RingDetector;

import Library4997.MasqVision.MasqCamera;
import Library4997.MasqResources.MasqMath.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker;
import Library4997.MasqRobot;
import Library4997.MasqWrappers.DashBoard;

import static Library4997.MasqVision.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqMotors.MasqMotorModel.*;
import static Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqResources.MasqUtils.*;
import static Library4997.MasqWrappers.DashBoard.getDash;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class PlaceHolder extends MasqRobot {
    public MasqCamera camera;
    public MasqMotor intake, encoder1, encoder2, shooter;
    public RingDetector detector;
    public RotatingClaw claw;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException{
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);

        shooter = new MasqMotor("shooter", NEVERREST37, REVERSE, hardwareMap);
        intake = new MasqMotor("intake", USDIGITAL_E4T, hardwareMap);

        claw = new RotatingClaw(hardwareMap);

        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, REVERSE, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);

        dash = getDash();
    }

    public void init(HardwareMap hardwareMap) throws InterruptedException{
        mapHardware(hardwareMap);

        shooter.setClosedLoop(true);
        shooter.reverseEncoder();
        shooter.setKp(1e-8);

        intake.setWheelDiameter(2);
        encoder1.setWheelDiameter(2);
        encoder2.setWheelDiameter(2);
        shooter.setWheelDiameter(4);

        shooter.resetEncoder();
        intake.resetEncoder();
        encoder1.resetEncoder();
        encoder2.resetEncoder();

        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);
        tracker.setPosition(THREE);
        tracker.setXRadius(5.675);
        tracker.setTrackWidth(13.75);

        driveTrain.setTracker(tracker);
        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.003);
        xySpeedController = new MasqPIDController(0.08);
        xyAngleController = new MasqPIDController(0.09);

        driveTrain.setClosedLoop(true);
        driveTrain.setKp(5e-8);
        driveTrain.resetEncoders();

        claw.reset();
    }

    public void initCamera(HardwareMap hardwareMap) {
        detector = new RingDetector();
        detector.setClippingMargins(600,360,250,750);
        camera = new MasqCamera(detector,WEBCAM, hardwareMap);
        camera.start();
    }
}
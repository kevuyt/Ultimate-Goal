package org.firstinspires.ftc.teamcode.Osiris.Robot;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;

import Library4997.MasqServos.MasqServo;
import Library4997.MasqVision.MasqCamera;
import Library4997.MasqResources.MasqMath.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker;
import Library4997.MasqRobot;

import static Library4997.MasqVision.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqMotors.MasqMotorModel.*;
import static Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqResources.MasqUtils.*;
import static Library4997.MasqWrappers.DashBoard.getDash;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class Osiris extends MasqRobot {
    public MasqCamera camera;
    public MasqMotor intake, encoder1, encoder2, shooter;
    public RingDetector detector;
    public RotatingClaw claw;
    public MasqServo flicker;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException{
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);

        shooter = new MasqMotor("shooter", NEVERREST37, DcMotorSimple.Direction.REVERSE, hardwareMap);
        intake = new MasqMotor("intake", USDIGITAL_E4T, hardwareMap);

        claw = new RotatingClaw(hardwareMap);

        flicker = new MasqServo("flicker", hardwareMap);

        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, DcMotorSimple.Direction.REVERSE, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);

        dash = getDash();
    }

    @Override
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
        driveTrain.setKp(9e-8);
        driveTrain.resetEncoders();

        claw.reset();
        flicker.setDirection(Servo.Direction.REVERSE);
        flicker.scaleRange(0.65, 0.86);
        flicker.setPosition(0);

    }

    public void initCamera(HardwareMap hardwareMap) {
        detector = new RingDetector();
        detector.setClippingMargins(600,360,250,750);
        camera = new MasqCamera(detector,WEBCAM, hardwareMap);
        camera.start();
    }
}
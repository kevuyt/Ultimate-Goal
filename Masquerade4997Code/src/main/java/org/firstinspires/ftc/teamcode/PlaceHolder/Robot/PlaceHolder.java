package org.firstinspires.ftc.teamcode.PlaceHolder.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PlaceHolder.Autonomous.Vision.RingDetector;

import Library4997.MasqCV.MasqCamera;
import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqPositionTracker;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqClock;

import static Library4997.MasqCV.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqMotors.MasqMotorModel.*;
import static Library4997.MasqPositionTracker.DeadWheelPosition.THREE;
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
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);

        shooter = new MasqMotor("shooter", NEVERREST_CLASSIC, REVERSE, hardwareMap);
        intake = new MasqMotor("intake", USDIGITAL_E4T, REVERSE, hardwareMap);

        claw = new RotatingClaw(hardwareMap);

        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, REVERSE, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);

        dash = getDash();
    }
 
    public void init(HardwareMap hardwareMap) {

        mapHardware(hardwareMap);
        intake.setWheelDiameter(2);
        encoder1.setWheelDiameter(2);
        encoder2.setWheelDiameter(2);
        shooter.setWheelDiameter(4);
        shooter.resetEncoder();

        tracker.setPosition(THREE);
        tracker.setXRadius(5.68);
        tracker.setTrackWidth(13.75);

        driveTrain.setTracker(tracker);
        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.003);
        xySpeedController = new MasqPIDController(0.12);
        xyAngleController = new MasqPIDController(0.09);

        driveTrain.setClosedLoop(true);
        driveTrain.setKp(1e-7);
        driveTrain.resetEncoders();

        claw.reset();
    }

    public void initCamera(HardwareMap hardwareMap) {
        detector = new RingDetector();
        detector.setClippingMargins(600,360,250,750);
        camera = new MasqCamera(detector,WEBCAM, hardwareMap);
        camera.start();
    }
    public void shoot(double power) {
        intake.setVelocity(1);
        sleep(0.25);
        shooter.setVelocity(power);
        while(shooter.getInches() < 12) {
            sleep(0.1);
        }
        shooter.setVelocity(0);
        shooter.resetEncoder();
    }
}
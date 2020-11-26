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
import Library4997.MasqServos.MasqServo;

import static Library4997.MasqCV.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqMotors.MasqMotorModel.*;
import static Library4997.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqResources.MasqUtils.*;
import static Library4997.MasqSensors.MasqClock.Resolution.SECONDS;
import static Library4997.MasqWrappers.DashBoard.getDash;


/**
 * Created by Keval Kataria on 9/12/2020
 */
public class PlaceHolder extends MasqRobot {
    public MasqCamera camera;
    public MasqMotor intake, encoder1, encoder2, shooter;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);
        intake = new MasqMotor("intake", USDIGITAL_E4T, hardwareMap);
        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, hardwareMap);
        shooter = new MasqMotor("shooter", NEVERREST_CLASSIC, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);
        dash = getDash();
    }

    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        tracker.setPosition(THREE);
        tracker.setXRadius(5.68);
        tracker.setTrackWidth(13.75);

        driveTrain.setTracker(tracker);
        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.003);
        velocityTeleController = new MasqPIDController(0.0000001);
        velocityAutoController = new MasqPIDController(0.0000001);
        xySpeedController = new MasqPIDController(0.08);
        xyAngleController = new MasqPIDController(0.06);

        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
    }

    public void initCamera(HardwareMap hardwareMap) {
        RingDetector detector = new RingDetector();
        detector.setClippingMargins(500,350,250,600);
        camera = new MasqCamera(detector,WEBCAM, hardwareMap);
        camera.start();
    }

    public void stop(double time) {
        MasqClock clock = new MasqClock();
        while(!clock.elapsedTime(time, SECONDS) && opModeIsActive()) {
            driveTrain.setVelocity(0);
        }
        driveTrain.setPower(0);
    }
    public void shoot(double power) {
        shooter.setVelocity(power);
        while(shooter.getInches() < 12) {
            sleep(0.1);
        }
        shooter.setVelocity(0);
        shooter.resetEncoder();
    }
}
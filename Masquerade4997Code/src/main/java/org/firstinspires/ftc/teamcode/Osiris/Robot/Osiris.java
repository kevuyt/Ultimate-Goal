package org.firstinspires.ftc.teamcode.Osiris.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMath.MasqPIDController;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker;
import Library4997.MasqSensors.MasqRangeSensor;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqVision.MasqCVDetector;
import Library4997.MasqVision.MasqCamera;

import static Library4997.MasqMotors.MasqMotorModel.*;
import static Library4997.MasqResources.DashBoard.getDash;
import static Library4997.MasqRobot.OpMode.AUTO;
import static Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqUtils.*;
import static Library4997.MasqVision.MasqCamera.Cam.WEBCAM;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_RIGHT;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class Osiris extends MasqRobot {
    public MasqCamera camera;
    public MasqMotor intake, encoder1, encoder2, shooter;
    public RotatingClaw claw;
    public MasqServo flicker, hopper;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);

        shooter = new MasqMotor("shooter", REVHDHEX1, hardwareMap);
        intake = new MasqMotor("intake", USDIGITAL_E4T, hardwareMap);

        claw = new RotatingClaw(hardwareMap);

        flicker = new MasqServo("flicker", hardwareMap);
        hopper = new MasqServo("hopper", hardwareMap);

        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);

        dash = getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap, OpMode opmode) {
        mapHardware(hardwareMap);

        tracker.setPosition(THREE);
        tracker.setXRadius(5.675);
        tracker.setTrackWidth(13.75);
        tracker.reset();

        setTracker(tracker);

        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.04);

        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();

        initServos();

        if(opmode == AUTO) {
            driveTrain.setKp(5e-8);
            initCamera(hardwareMap);
        }
        else driveTrain.setKp(5e-9);
    }

    public void initCamera(HardwareMap hardwareMap) {
        RingDetector detector = new RingDetector();
        detector.setClippingMargins(570,140,300,970);
        camera = new MasqCamera(detector, WEBCAM, hardwareMap);
        camera.start(SIDEWAYS_RIGHT);
    }

    private void initServos() {
        claw.reset();
        flicker.setDirection(Servo.Direction.REVERSE);
        flicker.scaleRange(0.05, 0.38);
        flicker.setPosition(0);
        hopper.scaleRange(0.05, 0.344);
        hopper.setPosition(0);
    }
    public int getRings() {
        return 0; //Placeholder until we get distance sensor
    }
}
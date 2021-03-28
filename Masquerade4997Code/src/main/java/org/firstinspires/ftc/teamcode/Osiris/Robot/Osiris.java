package org.firstinspires.ftc.teamcode.Osiris.Robot;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.RingDetector;

import MasqLibrary.MasqMath.MasqPIDController;
import MasqLibrary.MasqMotion.*;
import MasqLibrary.MasqOdometry.MasqPositionTracker;
import MasqLibrary.MasqRobot;
import MasqLibrary.MasqSensors.MasqDistanceSensor;
import MasqLibrary.MasqVision.MasqCamera;

import static MasqLibrary.MasqResources.MasqUtils.*;
import static MasqLibrary.MasqRobot.OpMode.AUTO;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;

/**
 * Created by Keval Kataria on 9/12/2020
 */

public class Osiris extends MasqRobot {
    public MasqCamera camera;
    public MasqMotor intake, encoder1, encoder2, shooter;
    public RotatingClaw claw;
    public MasqServo flicker, hopper, compressor;
    public MasqDistanceSensor distanceSensor;

    @Override
    public void mapHardware() {
        driveTrain = new MasqDriveTrain(REVERSE);

        shooter = new MasqMotor("shooter");
        intake = new MasqMotor("intake", REVERSE);

        claw = new RotatingClaw();

        flicker = new MasqServo("flicker");
        hopper = new MasqServo("hopper");
        compressor = new MasqServo("compressor");

        encoder1 = new MasqMotor("encoder1");
        encoder2 = new MasqMotor("encoder2", REVERSE);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2);

        distanceSensor = new MasqDistanceSensor("distanceSensor");

        dash = getDash();
    }
    @Override
    public void init(OpMode opmode) {
        mapHardware();

        tracker.setXRadius(5.675);
        tracker.setTrackWidth(13.75);
        tracker.reset();
        setTracker(tracker);

        turnController = new MasqPIDController(0.04);

        driveTrain.setVelocityControl(true);
        driveTrain.resetEncoders();

        initServos();

        shooter.setVelocityControl(true);

        if(opmode == AUTO) initCamera();
    }
    public void initCamera() {
        RingDetector detector = new RingDetector();
        detector.setClippingMargins(662,324,208,786);
        camera = new MasqCamera(detector);
        camera.start(SIDEWAYS_LEFT);
    }
    private void initServos() {
        claw.reset();
        flicker.scaleRange(0.07 , 0.21);
        flicker.setPosition(0);
        hopper.scaleRange(0.52, 0.855);
        hopper.setPosition(0);
        compressor.scaleRange(0.125, 0.413);
        compressor.setPosition(1);
    }

    public int getRings() {return 0;} //Placeholder until I get distance sensor
}
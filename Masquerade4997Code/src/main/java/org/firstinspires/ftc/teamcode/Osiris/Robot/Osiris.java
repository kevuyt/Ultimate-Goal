package org.firstinspires.ftc.teamcode.Osiris.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;

import MasqueradeLibrary.MasqMath.MasqPIDController;
import MasqueradeLibrary.MasqMotion.*;
import MasqueradeLibrary.MasqPositionTracker;
import MasqueradeLibrary.MasqRobot;
import MasqueradeLibrary.MasqVision.MasqCamera;

import static MasqueradeLibrary.MasqMotion.MasqMotor.MasqMotorModel.*;
import static MasqueradeLibrary.MasqResources.DashBoard.getDash;
import static MasqueradeLibrary.MasqResources.MasqUtils.*;
import static MasqueradeLibrary.MasqRobot.OpMode.AUTO;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;

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
        driveTrain = new MasqDriveTrain(hardwareMap);

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

        tracker.setXRadius(5.675);
        tracker.setTrackWidth(13.75);
        tracker.reset();

        setTracker(tracker);

        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.04);

        driveTrain.setVelocityControl(true);
        driveTrain.resetEncoders();

        initServos();

        shooter.setVelocityControl(true);

        if(opmode == AUTO) initCamera(hardwareMap);
    }

    public void initCamera(HardwareMap hardwareMap) {
        RingDetector detector = new RingDetector();
        detector.setClippingMargins(662,324,208,786);
        camera = new MasqCamera(detector, hardwareMap);
        camera.start(SIDEWAYS_LEFT);
    }

    private void initServos() {
        claw.reset();
        flicker.scaleRange(0.045, 0.18);
        flicker.setPosition(0);
        hopper.scaleRange(0.05, 0.344);
        hopper.setPosition(0);
    }
    public int getRings() {return 0;} //Placeholder until I get distance sensor
}
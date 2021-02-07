package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Osiris.Autonomous.Vision.RingDetector;
import org.firstinspires.ftc.teamcode.Osiris.Robot.RotatingClaw;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqMath.MasqPIDController;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker;
import Library4997.MasqVision.MasqCamera;

import static Library4997.MasqMotors.MasqMotorModel.NEVERREST37;
import static Library4997.MasqMotors.MasqMotorModel.REVHDHEX20;
import static Library4997.MasqMotors.MasqMotorModel.USDIGITAL_E4T;
import static Library4997.MasqResources.MasqUtils.angleController;
import static Library4997.MasqResources.MasqUtils.driveController;
import static Library4997.MasqResources.MasqUtils.turnController;
import static Library4997.MasqResources.MasqUtils.xyAngleController;
import static Library4997.MasqResources.MasqUtils.xySpeedController;
import static Library4997.MasqSensors.MasqPositionTracker.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqVision.MasqCamera.Cam.WEBCAM;
import static Library4997.MasqWrappers.DashBoard.getDash;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Keval Kataria on 9/12/2020
 */
public class TestBot extends MasqRobot {
    public MasqCamera camera;
    public RingDetector detector;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException{
        dash = getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap) throws InterruptedException{
        mapHardware(hardwareMap);
    }

    public void initCamera(HardwareMap hardwareMap) {
        detector = new RingDetector();
        detector.setClippingMargins(600,360,250,750);
        camera = new MasqCamera(detector,WEBCAM, hardwareMap);
        camera.start();
    }
}
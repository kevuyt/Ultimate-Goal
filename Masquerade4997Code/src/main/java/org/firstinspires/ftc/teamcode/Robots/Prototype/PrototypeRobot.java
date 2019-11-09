package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SkystoneDetection.DogeDetector;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class PrototypeRobot extends MasqRobot {

    MasqServo blockGrabber, blockRotater, blockPusher, sideGrabber;
    MasqServoSystem foundationHook;
    MasqMotor  lift;
    MasqMotorSystem intake;
    DogeDetector detector;
    private BNO055IMU imu;

    @Override
    public void init(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE,MasqMotorModel.REVHDHEX40, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);
        imu = initializeIMU(imu, hardwareMap);
        tracker = new MasqPositionTracker(lift, intake.motor1, imu); //Replace motors when odometry is incorporating
        foundationHook = new MasqServoSystem("rightGrabber", "leftGrabber", hardwareMap);
        detector = new DogeDetector(DogeDetector.Cam.PHONE, hardwareMap);
        sideGrabber = new MasqServo("sideGrabber", hardwareMap);

        blockPusher.scaleRange(0,0.5);
        blockGrabber.scaleRange(0,0.5);
        blockRotater.scaleRange(0.02,0.7);
        foundationHook.scaleRange(0,1);
        sideGrabber.scaleRange(0,1);

        lift.encoder.setWheelDiameter(1);

    }

    public void resetServos() {
        blockPusher.setPosition(0);
        blockRotater.setPosition(0);
        blockGrabber.setPosition(1);
        lowerFoundationHook();
        sideGrabber.setPosition(0);
    }

    public void lowerFoundationHook() {
        foundationHook.servo1.setPosition(1);
        foundationHook.servo2.setPosition(0);
    }
    public void raiseFoundationHook() {
        foundationHook.servo1.setPosition(0);
        foundationHook.servo2.setPosition(1);
    }
    public void midFoundationHook() {
        foundationHook.servo1.setPosition(0.2);
        foundationHook.servo2.setPosition(0.8);
    }

    public void runStoneLeft(HardwareMap hardwareMap) {
        //Collect and place on foundation first and fourth blocks
    }
    public void runStoneMiddle(HardwareMap hardwareMap) {
        //Collect and place on foundation second and fifth blocks
    }
    public void runStoneRight(HardwareMap hardwareMap) {
        //Collect and place on foundation third and sixth blocks
    }


}
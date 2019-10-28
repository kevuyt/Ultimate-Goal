package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.OpenCV.DogeDetector;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqServos.MasqServo;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class PrototypeRobot extends MasqRobot {

    MasqServo blockGrabber, blockRotater, blockPusher, foundationHook;
    MasqMotor  lift;
    MasqMotorSystem intake;
    DogeDetector detector;
    private DogeDetector.SkystonePosition skystonePosition;

    @Override
    public void init(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE,MasqMotorModel.REVHDHEX40, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);
        MasqAdafruitIMU imu = new MasqAdafruitIMU("imu", hardwareMap);
        tracker = new MasqPositionTracker(driveTrain.leftDrive,driveTrain.rightDrive, imu);
        foundationHook = new MasqServo("foundationHook", hardwareMap);
        detector = new DogeDetector(DogeDetector.Cam.WEBCAM, hardwareMap);

        blockPusher.scaleRange(0,0.5);
        blockGrabber.scaleRange(0,0.5);
        blockRotater.scaleRange(0.02,0.7);
        foundationHook.scaleRange(0,1);

        lift.encoder.setWheelDiameter(1);

    }

    @Override
    public MasqPIDPackage pidPackage() {return new MasqPIDPackage();}

    public void runStoneNotDetectedAuto(HardwareMap hardwareMap) {
        skystonePosition = null;
    }
    public void runStoneLeft(HardwareMap hardwareMap) {}
    public void runStoneMiddle(HardwareMap hardwareMap) {}
    public void runStoneRight(HardwareMap hardwareMap) {}
}
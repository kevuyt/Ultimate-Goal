package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqResources.MasqHelpers.MasqEncoderModel;
import Library4997.MasqWrappers.DashBoard;
import SubSystems4997.MasqRobot;
import SubSystems4997.SubSystems.Flipper;
import SubSystems4997.SubSystems.Gripper;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Creed extends MasqRobot {
    public MasqMotorSystem intake;
    public MasqMotor lift, relicLift;
    public MasqServo redRotator;
    public MasqREVColorSensor jewelColorRed;
    public Flipper flipper;
    public Gripper gripper;
    public MasqServo relicAdjuster;
    public MasqVoltageSensor voltageSensor;
    public MasqServo jewelArmRed, relicGripper;
    public MasqVuforiaBeta vuforia;
    public MasqAdafruitIMU imu;
    public MasqREVColorSensor singleBlock, doubleBlock, redLineDetector, blueLineDetector;
    public void mapHardware(HardwareMap hardwareMap) {
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        vuforia = new MasqVuforiaBeta();
        blueLineDetector = new MasqREVColorSensor("blueLineDetector", hardwareMap);
        redLineDetector = new MasqREVColorSensor("redLineDetector", hardwareMap);
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", hardwareMap);
        doubleBlock = new MasqREVColorSensor("doubleBlock", hardwareMap);
        singleBlock = new MasqREVColorSensor("singleBlock", hardwareMap);
        voltageSensor = new MasqVoltageSensor(hardwareMap);
        flipper = new Flipper(hardwareMap);
        redRotator = new MasqServo("redRotator", hardwareMap);
        lift = new MasqMotor("lift", MasqEncoderModel.NEVEREST40, DcMotor.Direction.REVERSE, hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        relicAdjuster = new MasqServo("relicAdjuster", hardwareMap);
        jewelArmRed = new MasqServo("jewelArmRed", hardwareMap);
        jewelColorRed = new MasqREVColorSensor("jewelColorRed", hardwareMap);
        relicGripper = new MasqServo("relicGripper", hardwareMap);
        relicLift = new MasqMotor("relicLift", MasqEncoderModel.NEVEREST40, hardwareMap);
        gripper = flipper.getGripper();
        tracker = new MasqPositionTracker(driveTrain.leftDrive, driveTrain.rightDrive, imu);
        lift.setClosedLoop(false);
    }

}

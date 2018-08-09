package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqControlSystems.MasqPositionTracker;
import Library4997.MasqSensors.MasqREVColorSensor;
import Library4997.MasqSensors.MasqVoltageSensor;
import Library4997.MasqSensors.MasqVuforiaBeta;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqResources.MasqHelpers.MasqEncoderModel;
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
    public MasqREVColorSensor singleBlock, doubleBlock, redLineDetector, blueLineDetector;
    HardwareMap hardwareMap;
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        vuforia = new MasqVuforiaBeta();
        blueLineDetector = new MasqREVColorSensor("blueLineDetector", this.hardwareMap);
        redLineDetector = new MasqREVColorSensor("redLineDetector", this.hardwareMap);
        intake = new MasqMotorSystem("leftIntake", DcMotor.Direction.REVERSE, "rightIntake", DcMotor.Direction.FORWARD, "INTAKE", this.hardwareMap);
        doubleBlock = new MasqREVColorSensor("doubleBlock", this.hardwareMap);
        singleBlock = new MasqREVColorSensor("singleBlock", this.hardwareMap);
        voltageSensor = new MasqVoltageSensor(this.hardwareMap);
        flipper = new Flipper(this.hardwareMap);
        redRotator = new MasqServo("redRotator", this.hardwareMap);
        lift = new MasqMotor("lift", DcMotor.Direction.REVERSE, this.hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(this.hardwareMap);
        relicAdjuster = new MasqServo("relicAdjuster", this.hardwareMap);
        jewelArmRed = new MasqServo("jewelArmRed", this.hardwareMap);
        jewelColorRed = new MasqREVColorSensor("jewelColorRed", this.hardwareMap);
        relicGripper = new MasqServo("relicGripper", this.hardwareMap);
        relicLift = new MasqMotor("relicLift", this.hardwareMap);
        gripper = flipper.getGripper();
        positionTracker = new MasqPositionTracker(hardwareMap, relicLift, MasqEncoderModel.USDIGITAL_E4T,
                driveTrain.rightDrive.motor2, MasqEncoderModel.NEVEREST20);
        positionTracker.xWheel.setWheelDiameter(4);
        positionTracker.resetSystem();
        lift.setClosedLoop(false);
    }
}

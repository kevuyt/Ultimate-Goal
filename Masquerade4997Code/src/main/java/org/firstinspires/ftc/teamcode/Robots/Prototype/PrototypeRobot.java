package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class PrototypeRobot extends MasqRobot {

    MasqServo blockGrabber, blockRotater, blockPusher;
    MasqMotor intakeRight, intakeLeft;
    MasqMotor lift;

    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        blockGrabber = new MasqServo("blockGrabber", hardwareMap);
        lift = new MasqMotor("lift",MasqMotorModel.ORBITAL20, hardwareMap);
        blockRotater = new MasqServo("blockRotater", hardwareMap);
        intakeRight = new MasqMotor("intakeRight", MasqMotorModel.REVHDHEX, hardwareMap);
        intakeLeft = new MasqMotor("intakeLeft", MasqMotorModel.REVHDHEX, hardwareMap);
        blockPusher = new MasqServo("blockPusher", hardwareMap);

    }

    @Override
    public MasqPIDPackage pidPackage() {return new MasqPIDPackage();}
}

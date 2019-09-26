package org.firstinspires.ftc.teamcode.Robots.Prototype;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class PrototypeRobot extends MasqRobot {

    MasqServo gripper, blockRotater;
    MasqMotorSystem intake;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        gripper = new MasqServo("gripper", hardwareMap);
        blockRotater = new MasqServo("block Rotater", hardwareMap);
        intake = new MasqMotorSystem("intakeLeft", DcMotorSimple.Direction.FORWARD, "intakeRight", DcMotorSimple.Direction.REVERSE,MasqMotorModel.REVHDHEX, hardwareMap);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}

package org.firstinspires.ftc.teamcode.Robots.Robot1.Prototype;

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

    public MasqServo Grabber, Twister;
    public MasqMotorSystem Intake;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain("leftFront", "leftBack", "rightFront", "rightBack", hardwareMap);
        Grabber = new MasqServo("Yes", hardwareMap);
        Twister = new MasqServo("No", hardwareMap);
        Intake = new MasqMotorSystem("gripperLeft", DcMotorSimple.Direction.FORWARD, "gripperRight", DcMotorSimple.Direction.REVERSE,MasqMotorModel.REVHDHEX, hardwareMap);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}

package org.firstinspires.ftc.teamcode.Robots.Kenya;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 3/15/19.
 * Project: MasqLib
 */
public class Kenya extends MasqRobot {
    public MasqDriveTrain driveTrain;
    public MasqMotor test;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        //driveTrain = new MasqDriveTrain(hardwareMap);
        test = new MasqMotor("test", MasqMotorModel.NEVEREST40, hardwareMap);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}

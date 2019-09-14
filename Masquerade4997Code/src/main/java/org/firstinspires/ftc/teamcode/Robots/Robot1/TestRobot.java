package org.firstinspires.ftc.teamcode.Robots.Robot1;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 2019-08-06.
 * Project: MasqLib
 */
public class TestRobot extends MasqRobot {
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain("leftFront", "leftBack", "rightFront", "rightBack", hardwareMap);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}

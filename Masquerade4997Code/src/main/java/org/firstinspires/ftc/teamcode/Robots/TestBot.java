package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqDriveTrains.MasqDriveTrain;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class TestBot extends MasqRobot {
    private HardwareMap hardwareMap;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        driveTrain = new MasqDriveTrain(this.hardwareMap);
    }
}

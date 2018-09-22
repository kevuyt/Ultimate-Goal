package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqSensors.MasqPixy;
import Library4997.MasqWrappers.DashBoard;
import SubSystems4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 6/2/18.
 * Project: MasqLib
 */

public class Thanos extends MasqRobot {
    public MasqAdafruitIMU imu;
    public MasqPixy pixy;
    public void mapHardware(HardwareMap hardwareMap) {
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqDriveTrain(hardwareMap);
        tracker = new MasqPositionTracker(driveTrain.leftDrive, driveTrain.rightDrive, imu);
        pixy = new MasqPixy(hardwareMap);
    }
    public void update () {
        dash.update();
        tracker.updateSystemV2();
        pixy.update();
    }

}

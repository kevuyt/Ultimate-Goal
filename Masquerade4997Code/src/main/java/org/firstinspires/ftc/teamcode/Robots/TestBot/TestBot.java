package org.firstinspires.ftc.teamcode.Robots.TestBot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqPIDPackage;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 10/13/18.
 * Project: MasqLib
 */

public class TestBot extends MasqRobot{
    public MasqAdafruitIMU imu;
    public void mapHardware(HardwareMap hardwareMap) {
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqDriveTrain(hardwareMap, MasqMotorModel.NEVEREST40);
        tracker = new MasqPositionTracker(driveTrain.leftDrive, driveTrain.rightDrive, imu);
    }
    @Override
    public MasqPIDPackage pidPackage() {
        return new MasqPIDPackage();
    }
}

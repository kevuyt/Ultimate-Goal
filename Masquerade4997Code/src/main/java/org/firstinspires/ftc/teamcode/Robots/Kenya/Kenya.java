package org.firstinspires.ftc.teamcode.Robots.Kenya;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqServos.MasqServoSystem;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 3/15/19.
 * Project: MasqLib
 */
public class Kenya extends MasqRobot {
    public MasqServoSystem jeff;
    public MasqServo wing1, wing2;
    public MasqAdafruitIMU imu;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        wing1 = new MasqServo("wing1", hardwareMap);
        wing2 = new MasqServo("wing2", hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(hardwareMap);
        jeff = new MasqServoSystem(wing1, wing2);
        dash = DashBoard.getDash();
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        tracker = new MasqPositionTracker(driveTrain.leftDrive.motor1, driveTrain.rightDrive.motor2, imu);
    }

    @Override
    public MasqPIDPackage pidPackage() {
        MasqPIDPackage pidPackage = new MasqPIDPackage();
        pidPackage.setKpTurn(0.001);
        return new MasqPIDPackage();
    }
}

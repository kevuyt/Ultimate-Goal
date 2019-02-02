package org.firstinspires.ftc.teamcode.Robots.Frenzy;

import com.qualcomm.robotcore.hardware.HardwareMap;
import Library4997.MasqControlSystems.MasqPurePursuit.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqControlSystems.MasqPID.MasqPIDPackage;
import Library4997.MasqRobot;
import Library4997.MasqSensors.MasqAdafruitIMU;
import Library4997.MasqServos.MasqCRServo;
import Library4997.MasqWrappers.DashBoard;

/**
 * Created by Archishmaan Peyyety on 1/5/19.
 * Project: MasqLib
 */
public class Frenzy extends MasqRobot {
    public MasqMotor
            actuator,
            pivot,
            wrist,
            horzLift;
    public MasqCRServo
            leftIntake,
            rightIntake;
    public MasqAdafruitIMU imu;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        imu = new MasqAdafruitIMU("imu", hardwareMap);
        driveTrain = new MasqDriveTrain("driveLF", "driveLB", "driveRF", "driveRB", hardwareMap);
        tracker = new MasqPositionTracker(driveTrain.leftDrive, driveTrain.rightDrive, imu);
        actuator = new MasqMotor("actuator", MasqMotorModel.ORBITAL20, hardwareMap);
        pivot = new MasqMotor("pivot", MasqMotorModel.NEVEREST60, hardwareMap);
        wrist = new MasqMotor("wrist", MasqMotorModel.NEVEREST60, hardwareMap);
        horzLift = new MasqMotor("horzLift", MasqMotorModel.ORBITAL20, hardwareMap);
        leftIntake = new MasqCRServo("leftIntake", hardwareMap);
        rightIntake = new MasqCRServo("rightIntake", hardwareMap);
        dash = DashBoard.getDash();
    }

    @Override
    public MasqPIDPackage pidPackage() {
        MasqPIDPackage pidPackage = new MasqPIDPackage();
        pidPackage.setKpTurn(0.01);
        return new MasqPIDPackage();
    }
}

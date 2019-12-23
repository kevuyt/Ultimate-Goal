package org.firstinspires.ftc.teamcode.Robots.TestRobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqPositionTracker;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqResources.MasqHelpers.MasqMotorModel;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;

/**
 * Created by Keval Kataria on 9/25/2019
 */
public class TestRobot extends MasqRobot {
    //public MasqMotorSystem intake;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, MasqMotorModel.ORBITAL20);
        tracker = new MasqPositionTracker(driveTrain.leftDrive.motor1, driveTrain.rightDrive.motor1,hardwareMap);
        //intake = new MasqMotorSystem("intakeRight", DcMotorSimple.Direction.FORWARD, "intakeLeft", DcMotorSimple.Direction.REVERSE, MasqMotorModel.ORBITAL20,hardwareMap);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        MasqUtils.driveController = new MasqPIDController(0.005,0,0);
        MasqUtils.angleController = new MasqPIDController(0.01,0,0);
        MasqUtils.turnController = new MasqPIDController(0.015,0,0);
        MasqUtils.velocityTeleController = new MasqPIDController(0.002, 0, 0);
        MasqUtils.velocityAutoController = new MasqPIDController(0.002, 0, 0);
        driveTrain.setClosedLoop(true);
    }
}
package org.firstinspires.ftc.teamcode.Robots.Midnight.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.SubSystems.FoundationHook;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqPositionTracker;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
public class Midnight extends MasqRobot {
    public FoundationHook foundationHook;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        foundationHook = new FoundationHook(hardwareMap);
        MasqMotorSystem left = new MasqMotorSystem("left", MasqMotorModel.ORBITAL20, DcMotor.Direction.FORWARD, hardwareMap);
        MasqMotorSystem right = new MasqMotorSystem("right", MasqMotorModel.ORBITAL20, DcMotor.Direction.REVERSE, hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(left, right);
        tracker = new MasqPositionTracker(hardwareMap);
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        MasqUtils.driveController = new MasqPIDController(0.005);
        MasqUtils.angleController = new MasqPIDController(0.005);
        MasqUtils.turnController = new MasqPIDController(0.015);
        MasqUtils.velocityTeleController = new MasqPIDController(0.001);
        MasqUtils.velocityAutoController = new MasqPIDController(0.004);
        MasqUtils.xySpeedController = new MasqPIDController(0.04, 0, 0);
        MasqUtils.xyAngleController = new MasqPIDController(0.05, 0, 0);
        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        scaleServos();
        resetServos();
    }
    public void scaleServos() {

    }
    public void resetServos() {

    }
}

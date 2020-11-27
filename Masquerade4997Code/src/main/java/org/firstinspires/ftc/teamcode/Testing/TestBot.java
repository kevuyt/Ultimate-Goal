package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqPositionTracker;
import Library4997.MasqRobot;

import static Library4997.MasqMotors.MasqMotorModel.REVHDHEX20;
import static Library4997.MasqMotors.MasqMotorModel.USDIGITAL_E4T;
import static Library4997.MasqPositionTracker.DeadWheelPosition.THREE;
import static Library4997.MasqResources.MasqUtils.angleController;
import static Library4997.MasqResources.MasqUtils.driveController;
import static Library4997.MasqResources.MasqUtils.turnController;
import static Library4997.MasqResources.MasqUtils.xyAngleController;
import static Library4997.MasqResources.MasqUtils.xySpeedController;
import static Library4997.MasqWrappers.DashBoard.getDash;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

/**
 * Created by Keval Kataria on 6/4/2020
 */
public class TestBot extends MasqRobot {
    MasqMotor intake, encoder1, encoder2;

    @Override
    public void mapHardware(HardwareMap hardwareMap) throws InterruptedException {
        driveTrain = new MasqMechanumDriveTrain(hardwareMap, REVHDHEX20);
        intake = new MasqMotor("intake", USDIGITAL_E4T, REVERSE, hardwareMap);
        encoder1 = new MasqMotor("encoder1", USDIGITAL_E4T, REVERSE, hardwareMap);
        encoder2 = new MasqMotor("encoder2", USDIGITAL_E4T, hardwareMap);
        tracker = new MasqPositionTracker(intake, encoder1, encoder2, hardwareMap);
        dash = getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap) throws InterruptedException {
        mapHardware(hardwareMap);
        intake.setWheelDiameter(2);
        encoder1.setWheelDiameter(2);
        encoder2.setWheelDiameter(2);

        tracker.setPosition(THREE);
        tracker.setXRadius(5.68);
        tracker.setTrackWidth(13.75);

        driveTrain.setTracker(tracker);
        driveController = new MasqPIDController(0.005);
        angleController = new MasqPIDController(0.003);
        turnController = new MasqPIDController(0.003);
        xySpeedController = new MasqPIDController(0.08);
        xyAngleController = new MasqPIDController(0.06);

        driveTrain.setClosedLoop(true);
        driveTrain.setKp(1e-7);
        driveTrain.resetEncoders();
    }
}
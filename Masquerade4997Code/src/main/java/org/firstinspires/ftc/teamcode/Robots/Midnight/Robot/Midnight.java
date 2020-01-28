package org.firstinspires.ftc.teamcode.Robots.Midnight.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.SubSystems.FoundationHook;

import Library4997.MasqControlSystems.MasqPID.MasqPIDController;
import Library4997.MasqDriveTrains.MasqMechanumDriveTrain;
import Library4997.MasqMotors.MasqMotor;
import Library4997.MasqMotors.MasqMotorModel;
import Library4997.MasqMotors.MasqMotorSystem;
import Library4997.MasqPositionTracker;
import Library4997.MasqResources.MasqUtils;
import Library4997.MasqRobot;
import Library4997.MasqServos.MasqServo;
import Library4997.MasqWrappers.DashBoard;

import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.CAP_STORE;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.GRAB_INIT;
import static org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Constants.PIVOT_INIT;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
public class Midnight extends MasqRobot {
    public FoundationHook foundationHook;
    public MasqServo grabber, pivot, capstone;
    public MasqMotor lift;
    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        capstone = new MasqServo("capstone", hardwareMap);
        pivot = new MasqServo("pivot", hardwareMap);
        lift = new MasqMotor("lift", MasqMotorModel.ORBITAL20, hardwareMap);
        grabber = new MasqServo("grabber", hardwareMap);
        foundationHook = new FoundationHook(hardwareMap);
        MasqMotorSystem left = new MasqMotorSystem("left", MasqMotorModel.ORBITAL20, DcMotor.Direction.REVERSE, hardwareMap);
        MasqMotorSystem right = new MasqMotorSystem("right", MasqMotorModel.ORBITAL20, DcMotor.Direction.FORWARD, hardwareMap);
        driveTrain = new MasqMechanumDriveTrain(left, right);
        tracker = new MasqPositionTracker(hardwareMap);
        dash = DashBoard.getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);
        MasqUtils.driveController = new MasqPIDController(0.0015);
        MasqUtils.angleController = new MasqPIDController(0.005);
        MasqUtils.turnController = new MasqPIDController(0.005);
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
        capstone.setPosition(CAP_STORE);
        foundationHook.hooksUp();
        pivot.setPosition(PIVOT_INIT);
        grabber.setPosition(GRAB_INIT);
    }
}

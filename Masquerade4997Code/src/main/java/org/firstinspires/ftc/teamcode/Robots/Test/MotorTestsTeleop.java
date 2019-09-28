package org.firstinspires.ftc.teamcode.Robots.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqMotors.MasqMotor_Tests;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "MotorTestsTeleop", group = "Test")
public class MotorTestsTeleop extends MasqLinearOpMode {
    private TestRobot robot = new TestRobot();
    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        MasqMotor_Tests masqMotorTests = new MasqMotor_Tests(robot.driveTrain.leftDrive.motor1);
        while (opModeIsActive()) {
            try {
                masqMotorTests.RunAll();

            }
            catch (AssertionError error)  {
                dash.create(error);
                dash.update();

            }
        }
    }
}


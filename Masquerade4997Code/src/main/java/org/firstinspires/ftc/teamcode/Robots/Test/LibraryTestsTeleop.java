package org.firstinspires.ftc.teamcode.Robots.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library4997.MasqMotors.MasqMotor_Tests;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "LibraryTestsTeleop", group = "Test")
public class LibraryTestsTeleop extends MasqLinearOpMode {
    private TestRobot robot = new TestRobot();
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }
        waitForStart();
        MasqMotor_Tests masqMotorTests = new MasqMotor_Tests(robot.driveTrain.leftDrive.motor1) {
            @Override
            public void RunAll(HardwareMap hardwareMap) {

            }
        };
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


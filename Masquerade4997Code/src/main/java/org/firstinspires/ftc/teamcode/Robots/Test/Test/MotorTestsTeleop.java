package org.firstinspires.ftc.teamcode.Robots.Robot1.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Robot1.Prototype.PrototypeRobot;

import Library4997.MasqMotors.MasqMotor_Tests;
import Library4997.MasqRobot;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "MotorTestsTeleop", group = "NFS")
public class MotorTestsTeleop extends MasqLinearOpMode {
    private MasqRobot robot = new PrototypeRobot();
    @Override
    public void runLinearOpMode() throws InterruptedException {
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

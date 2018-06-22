package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/18/18.
 * Project: MasqLib
 */

@Autonomous(name = "Gyro-TestBot", group = "Autonomus")
public class GyroTestBot extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.positionTracker.imu.getAbsoluteHeading());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive() && robot.positionTracker.imu.getAbsoluteHeading() < 90) {
            robot.leftMotor.setPower(1);
            robot.rightMotor.setPower(-1);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}

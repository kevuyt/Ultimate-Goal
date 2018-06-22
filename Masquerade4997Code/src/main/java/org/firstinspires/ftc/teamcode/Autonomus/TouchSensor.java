package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/18/18.
 * Project: MasqLib
 */

@Autonomous(name = "TouchSensor-TestBot", group = "Autonomus")
public class TouchSensor extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.touchSensor.isPressed());
            dash.update();
        }
        waitForStart();
        while (opModeIsActive() && robot.touchSensor.isPressed()) {
            robot.leftMotor.setPower(.75);
            robot.rightMotor.setPower(.75);
        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    public void stopDriving () {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}

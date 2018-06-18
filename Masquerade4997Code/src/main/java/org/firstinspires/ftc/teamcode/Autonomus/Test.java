package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/18/18.
 * Project: MasqLib
 */

@Autonomous(name = "Sample", group = "Autonomus")
public class Test extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("init");
            dash.update();
        }
        waitForStart();
        robot.leftMotor.setPower(1);
        robot.rightMotor.setPower(1);
        robot.sleep(3);
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
}

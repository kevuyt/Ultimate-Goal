package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/18/18.
 * Project: MasqLib
 */

@Autonomous(name = "AngularCorrection", group = "Autonomus")
public class AngularCorrection extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.positionTracker.imu.getAbsoluteHeading());
            dash.create(robot.leftMotor.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        robot.drive(250, 0.7);
    }
}

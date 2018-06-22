package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Robot;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 6/18/18.
 * Project: MasqLib
 */

@Autonomous(name = "SquareTestBot", group = "Autonomus")
public class SquareTestBot extends MasqLinearOpMode implements Constants {
    private Robot robot = new Robot();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.positionTracker.imu.getAbsoluteHeading());
            dash.create(robot.leftMotor.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        robot.drive(80, 0.7);
        robot.turnRelative(90, Direction.RIGHT);
        robot.drive(80, 0.7);
        robot.turnRelative(90, Direction.RIGHT);
        robot.drive(80, 0.7);
        robot.turnRelative(90, Direction.RIGHT);
        robot.drive(80, 0.7);
        robot.turnRelative(90, Direction.RIGHT);
    }
}

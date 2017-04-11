package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */

@Autonomous(name = "ElijahAuto", group = "Template")
public class ElijahAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.driveTrain.setPower(1,1);
        robot.sleep();
        robot.drive(40, POWER_OPTIMAL);
        robot.shooter.setPower(-1);
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.sleep();
        robot.indexer.setPosition(INDEXER_CLOSED);
        robot.sleep();
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.turn(30,Direction.RIGHT);
        robot.drive(10);
    }
}

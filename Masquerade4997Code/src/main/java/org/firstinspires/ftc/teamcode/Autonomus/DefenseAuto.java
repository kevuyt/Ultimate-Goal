package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */

@Autonomous(name = "Defense_Auto", group = "Template")
public class DefenseAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.drive(30);
        robot.shooter.setPower(-1);
        robot.sleep(1000);
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.turn(43, Direction.LEFT);
        robot.drive(270);
        robot.sleep();
        robot.drive(50, POWER_OPTIMAL, Direction.BACKWARD);
    }
}

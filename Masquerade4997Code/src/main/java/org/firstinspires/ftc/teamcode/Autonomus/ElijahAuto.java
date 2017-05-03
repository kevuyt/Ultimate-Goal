package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;
import Library4997.MasqWrappers.Direction;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */

@Autonomous(name = "Elihahahahahah-auto", group = "Template")
public class ElijahAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.collector.setPower(COLLECTOR_IN);
        robot.drive(40, POWER_OPTIMAL);
        robot.shooter.setPower(TARGET_POWER);
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.sleep(900);
        robot.indexer.setPosition(INDEXER_CLOSED);
        robot.sleep(900);
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.turn(50,Direction.RIGHT);
        robot.drive(30);
    }
}

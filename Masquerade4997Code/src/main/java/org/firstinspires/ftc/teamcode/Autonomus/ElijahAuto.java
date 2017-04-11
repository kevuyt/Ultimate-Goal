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
    private int delay = 0;
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create("Press A on gamepad1 to set a delay of one second press b to reset");
            if (controller1.apr()){
                delay += 1000;
            }
            else if (controller1.bpr()){
                delay = 0;
            }
            dash.create("DELAY: ", delay);
            dash.create(robot.imu);
            dash.createSticky(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(delay);
        robot.drive(40);
        robot.shooter.setPower(TARGET_POWER);
        robot.indexer.setPosition(INDEXER_OPENED);
        robot.sleep();
        robot.turn(30, Direction.RIGHT);
        robot.drive(15);

    }
}

package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.MasqRobot;

/**
 * Red Autonomous
 */

@Autonomous(name = "RedAuto", group = "REd")
public class RedAuto extends MasqLinearOpMode implements Constants {
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
        dash.createSticky(robot.imu);
        robot.setAllianceColor(MasqRobot.AllianceColor.RED);
    }
}
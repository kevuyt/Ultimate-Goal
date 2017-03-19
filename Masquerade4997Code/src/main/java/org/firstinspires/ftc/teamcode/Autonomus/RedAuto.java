package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqRobot.MasqRobot;

/**
 * Red Autonomous
 */

@Autonomous(name = "RedAuto", group = "Test")
public class RedAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        dash.createSticky(robot.imu);
        robot.setAllianceColor(MasqRobot.AllianceColor.RED);
    }
}
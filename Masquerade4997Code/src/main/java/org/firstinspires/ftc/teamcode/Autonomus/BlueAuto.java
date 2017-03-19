package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqRobot.Direction;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "BlueAuto", group = "Test")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.leftColor);

            dash.update();
        }
        waitForStart();
        robot.drive(POWER_OPTIMAL, 60, Direction.FORWARD);

    }
}
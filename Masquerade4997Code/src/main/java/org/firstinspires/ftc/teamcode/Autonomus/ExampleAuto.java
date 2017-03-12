package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.Direction;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "Example-Auto", group = "Test")
public class ExampleAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.leftColor);
            dash.update();
        }
        waitForStart();
        robot.turn(90, Direction.LEFT);
    }
}

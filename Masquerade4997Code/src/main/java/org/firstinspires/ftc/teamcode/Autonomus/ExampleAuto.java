package org.firstinspires.ftc.teamcode.Autonomus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqRobot.Direction;
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
        robot.drive(POWER_OPTIMAL, 33, Direction.FORWARD);
        robot.turn(45, Direction.RIGHT);
        robot.drive(POWER_HIGH, 100, Direction.FORWARD);
    }
}

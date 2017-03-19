package org.firstinspires.ftc.teamcode.Autonomus;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;
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
        robot.drive(33, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(100,POWER_OPTIMAL, Direction.FORWARD);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.TankDrive.Direction;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends MasqLinearOpMode {

    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
        }
        waitForStart();
        robot.drive(0.5, 60, Direction.FORWARD);
        robot.turn(90, Direction.LEFT);
    }
}

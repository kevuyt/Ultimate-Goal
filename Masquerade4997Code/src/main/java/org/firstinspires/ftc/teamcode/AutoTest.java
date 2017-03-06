package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.Direction;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends MasqLinearOpMode {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.ultra);
            dash.create(robot.leftColor);
        }
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("IMU", robot.imu.getHeading());
            controller1.left_stick_y();
            controller2.left_stick_y();
            telemetry.update();
        }
    }
}

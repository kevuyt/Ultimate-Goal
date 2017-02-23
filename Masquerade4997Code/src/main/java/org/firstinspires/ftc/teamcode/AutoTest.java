package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import BasicLib4997.MasqMotors.TankDrive.Direction;
import BasicLib4997.MasqLinearOpMode;



/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "AutoTest", group = "Test")
public class AutoTest extends MasqLinearOpMode {

    public void runLinearOpMode() throws InterruptedException {
        while (!isStarted()) {
            dashBoard.create("Status", "Initialized");
            dashBoard.create(robot.imu);
            dashBoard.update();
        }

        waitForStart();
        robot.drive(0.7, 90, Direction.FORWARD);
        robot.turn(0.7 ,90, Direction.FORWARD);

    }
}

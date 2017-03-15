package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqRobot.Direction;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "BlueAuto1", group = "Test")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.leftColor);

            dash.update();
        }
        robot.presser.setPower(0);
        robot.indexer.setPosition(0);
        waitForStart();
        double parralelAngle = robot.imu.getHeading();
        robot.drive(POWER_OPTIMAL, 33, Direction.BACKWARD);
        double endAngleTurn1 = robot.imu.getHeading();
        double deltaTurn1 = endAngleTurn1 - parralelAngle;
        robot.turn(135 - (int)deltaTurn1, Direction.LEFT, 3, 0.001, 0.0002, 0);
        robot.drive(POWER_OPTIMAL,77,Direction.FORWARD);
        robot.stopTouch(POWER_LOW, Direction.FORWARD, robot.backTouch);
        sleep(100);
        double endAngleTurn2 = robot.imu.getHeading();
        double deltaTurn2 = endAngleTurn2 - parralelAngle;
        robot.turn(180 - (int) deltaTurn2, Direction.RIGHT, 3, 0.0020, 0.0002, 0);
        robot.stopWhite(POWER_LOW, Direction.FORWARD);
        robot.stopBlue(POWER_LOW, Direction.FORWARD);
        robot.drive(POWER_LOW, 2, Direction.BACKWARD);
        double endAngleTurn3 = robot.imu.getHeading();
        double deltaTurn3 = endAngleTurn3 - parralelAngle;
        robot.presser.setPower(-1);
        robot.sleep(1500);
        robot.presser.setPower(1);
        robot.presser.setPower(0);
        robot.sleep(1200);
        robot.turn(180 - (int) deltaTurn3, Direction.RIGHT, 1, 0.0020, 0.0002, 0);
        robot.drive(POWER_LOW, 50, Direction.BACKWARD);
        robot.stopWhite(POWER_LOW, Direction.BACKWARD);
        robot.stopBlue(POWER_LOW, Direction.FORWARD);
    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * BlueAutoCenterVortexLeft
 */

@Autonomous(name = "BlueAutoCenterVortexLeft", group = "Blue")
public class BlueAutoCenterVortexLeft extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.imu.reset();
        double startAngle = robot.imu.getYaw();
        robot.colorRejection.setActiveMode();
        robot.rightColor.setPassiveMode();
        robot.drive(10);
        robot.turn((int) ((robot.imu.getHeading() - startAngle) + cornerTurn), Direction.LEFT);
        robot.drive(90);
        robot.turn((int) (robot.imu.getHeading() - startAngle), Direction.RIGHT);
        robot.stopBlue(robot.leftColor);
        robot.leftPresser.setPower(-1);
        robot.sleep();
        robot.leftPresser.setPower(1);
        robot.drive(40);
        robot.stopBlue(robot.leftColor);
        robot.leftPresser.setPower(-1);
        robot.sleep();
        robot.leftPresser.setPower(1);
        // ok so the first thing u do is u shoot da ball into da hole and then you do a backflip then you win $$$$$$$$//
    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;
import Library4997.MasqWrappers.Direction;

/**
 * Red Autonomous
 */

@Autonomous(name = "RedAutoCenterVortexLeft", group = "REd")
public class RedAutoCenterVortexLeft extends MasqLinearOpMode implements Constants {
    private int delay = 0;
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
        robot.turn((int) (robot.imu.getHeading() - startAngle),Direction.RIGHT);
        robot.stopRed(robot.leftColor);
        robot.leftPresser.setPower(-1);
        robot.sleep();
        robot.leftPresser.setPower(1);
        robot.drive(40);
        robot.stopRed(robot.leftColor);
        robot.leftPresser.setPower(-1);
        robot.sleep();
        robot.leftPresser.setPower(1);
    }

}

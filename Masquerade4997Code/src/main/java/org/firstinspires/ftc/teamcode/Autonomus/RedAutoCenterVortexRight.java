package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * BlueAutoCenterVortexLeft
 */

@Autonomous(name = "RedAutoCenterVortexRight", group = "Red")
public class RedAutoCenterVortexRight extends MasqLinearOpMode implements Constants {
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
        robot.turn((int) ((robot.imu.getHeading() - startAngle) + 47), Direction.RIGHT);
        robot.drive(90);
        robot.turn((int) (robot.imu.getHeading() - startAngle),Direction.LEFT);
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
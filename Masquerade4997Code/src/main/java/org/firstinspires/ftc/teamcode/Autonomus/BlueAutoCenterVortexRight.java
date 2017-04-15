package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * BlueAutoCenterVortexLeft
 */

@Autonomous(name = "BlueAutoCenterVortexRight", group = "Blue")
public class BlueAutoCenterVortexRight extends MasqLinearOpMode implements Constants {
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
        robot.turn((int) ((robot.imu.getYaw() - startAngle) + cornerTurn), Direction.RIGHT);
        robot.drive(90);
        robot.turn((int) (robot.imu.getYaw() + startAngle), Direction.RIGHT);
        robot.stopBlue(robot.rightColor, POWER_LOW);
        robot.leftPresser.setPower(-1);
        robot.sleep(1500);
        robot.leftPresser.setPower(1);
        robot.drive(40);
        robot.stopBlue(robot.rightColor, POWER_LOW);
        robot.leftPresser.setPower(-1);
        robot.sleep(1500);
        robot.leftPresser.setPower(1);

        //go forward
        shoot ball
        do a backflip
         spin in circle until someone dies
        eat archis ass
        win;
    }
}
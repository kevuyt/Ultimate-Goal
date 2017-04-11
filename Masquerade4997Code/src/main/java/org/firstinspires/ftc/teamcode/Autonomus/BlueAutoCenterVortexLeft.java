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
        robot.colorRejection.setActiveMode();
        robot.rightColor.setPassiveMode();
        robot.collector.setPower(-1);
        robot.turn(10, Direction.LEFT);
    }
}
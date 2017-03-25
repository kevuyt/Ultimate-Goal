package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * BlueAuto
 */

@Autonomous(name = "BlueAuto", group = "Blue")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(robot.leftColor);

            dash.update();
        }
        waitForStart();
        robot.drive(60);

    }
}
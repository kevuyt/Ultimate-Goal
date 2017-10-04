package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqRobot;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * This is a basic template copy and paste this class for any auto,
 * refactor the file name to match the auto class title
 */


@Autonomous(name = "MainAuto", group = "Auto")
public class RelicMainAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!MasqRobot.getInstance(null).opModeIsActive()) {
            dash.create(robot.imu);
            dash.create(controller1);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            dash.create(controller1);
        }
    }
}
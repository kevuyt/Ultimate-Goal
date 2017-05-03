package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TeleOp.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * This is a basic autonomous program to test the various autonomous functions.
 */

@Autonomous(name = "AutoTest", group = "Test")
//
public class AutoTest extends MasqLinearOpMode implements Constants{
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.limitSwitch);
            dash.update();
        }
        waitForStart();

    }
}

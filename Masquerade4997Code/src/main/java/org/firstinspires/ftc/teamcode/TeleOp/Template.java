package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * This is a basic template copy and paste this class for any TeleOp,
 * refactor the file name to match the TeleOp class title
 */

@TeleOp(name = "Template-TelOp", group = "Template")
@Disabled
public class Template extends MasqLinearOpMode implements Constants{
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.createSticky(controller1);
            dash.createSticky(controller2);
            dash.update();
        }
        waitForStart();
    }
}

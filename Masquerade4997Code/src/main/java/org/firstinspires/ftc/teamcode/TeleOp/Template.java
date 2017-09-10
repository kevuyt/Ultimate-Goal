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
    int i = 0;
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()){
            robot.NFS(controller1);
            i ++;
            dash.create("LOOP COUNT", i);
            // or robot.MECH(controller1);
            // or robot.TANK(controller1);
        }
    }
}

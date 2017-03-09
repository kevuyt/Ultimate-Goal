package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.Direction;

/**
 * This is a basic template copy and paste this class for any teleop,
 * refactor the file name to match the teleop class title
 */

@TeleOp(name = "Template-telop", group = "Test")
@Disabled
public class Template extends MasqLinearOpMode implements TeleOp_Constants{
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot);
            dash.update();
        }
        waitForStart();
        dash.createSticky(controller1);
        dash.createSticky(controller2);
        while (opModeIsActive()) {
        //Place all teleop code here
        }
    }
}

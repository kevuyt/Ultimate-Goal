package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.Direction;

/**
 * A Template to follow for all TeleOp Opmodes
 */

@Autonomous(name = "Template", group = "Test")
@Disabled
public class Template extends MasqLinearOpMode implements TeleOp_Constants{
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
        /**
        * Place all TelOpCode Here
        */
        }
    }
}

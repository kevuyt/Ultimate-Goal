package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Constants;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/24/17.
 */
@TeleOp(name = "MasqMotorTest", group = "Autonomus")
@Disabled
public class MasqMotorTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        robot.stop(robot.jewelColorBlue, 0.3, Direction.FORWARD, 1);
    }
}
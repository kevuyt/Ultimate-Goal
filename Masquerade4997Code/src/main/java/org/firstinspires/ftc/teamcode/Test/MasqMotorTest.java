package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Constants;

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
        robot.driveTrain.setClosedLoop(true);
        while (opModeIsActive()) {
            robot.driveTrain.setPower(0.5);
            dash.create("Power: ", robot.driveTrain.getPower());
            dash.create(robot.driveTrain);
            dash.update();
        }
    }
}
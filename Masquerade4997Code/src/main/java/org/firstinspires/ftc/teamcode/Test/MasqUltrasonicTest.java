package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/25/17.
 */
@TeleOp(name = "MasqUltrasonicTest", group = "Autonomus")
public class MasqUltrasonicTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.matiboxUltraSensor);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.NFS(controller1);
        }
    }
}
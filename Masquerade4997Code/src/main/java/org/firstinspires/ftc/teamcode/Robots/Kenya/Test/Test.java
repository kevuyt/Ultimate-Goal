package org.firstinspires.ftc.teamcode.Robots.Kenya.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Kenya.Kenya;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 3/15/19.
 * Project: MasqLib
 */
@TeleOp(name = "Test", group = "NFS")
public class Test extends MasqLinearOpMode {
    private Kenya kenya;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        kenya = new Kenya();
        kenya.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hola");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            kenya.test.setPower(1);
        }
    }
}

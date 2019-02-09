package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "NFS")
public class ConstantsProgrammer extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double dumper = 0;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.xOnPress()) {
                dumper += 0.01;
            }
            if (controller1.yOnPress()) {
                dumper -= 0.01;
            }
            controller1.update();
            falcon.dumper.setPosition(dumper);
            dash.create("Dumper: ", falcon.dumper.getPosition());
            dash.update();
        }

    }
}

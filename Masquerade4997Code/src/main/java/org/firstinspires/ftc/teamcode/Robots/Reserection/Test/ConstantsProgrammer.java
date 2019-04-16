package org.firstinspires.ftc.teamcode.Robots.Reserection.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "NFS")
public class ConstantsProgrammer extends MasqLinearOpMode {
    private Resurrection falcon = new Resurrection();
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
            falcon.particleDumper.setPosition(dumper);

            falcon.scoreLift.DriverControl(controller1);

            dash.create("Dumper: ", falcon.particleDumper.getPosition());
            dash.update();
        }

    }
}

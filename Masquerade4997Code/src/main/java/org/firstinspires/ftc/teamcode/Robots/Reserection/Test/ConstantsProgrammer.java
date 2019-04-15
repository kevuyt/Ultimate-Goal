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
    private Resurrection resurrection = new Resurrection();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        resurrection.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
    }
}

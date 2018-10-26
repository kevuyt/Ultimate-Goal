package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "MainAUtoV1", group = "NFS")

public class MainAutoV1 extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double hangMin = 1, hangMax = 0.54;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.hangLatch.setPosition(hangMax);
        while (!opModeIsActive()) {
            dash.create("Hola");
            dash.update();
        }
        waitForStart();
        falcon.hangLatch.setPosition(hangMin);
        falcon.sleep(1);
    }
}

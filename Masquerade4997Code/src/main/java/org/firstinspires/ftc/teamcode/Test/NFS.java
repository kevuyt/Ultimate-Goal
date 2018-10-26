package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.TestBot;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "TestBot", group = "NFS")
public class NFS extends MasqLinearOpMode {
    private TestBot falcon = new TestBot();
    private double adjusterPosition = 0;
    private double adjusterMax = 1;
    private double adjusterMin = 0;
    private double dumperMin = 0.84;
    private double dumperMax = 0.2;
    private double adjusterIncrement = 0.05;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            falcon.NFS(controller1);
        }
    }
}

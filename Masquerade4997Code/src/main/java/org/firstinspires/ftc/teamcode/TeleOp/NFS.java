package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Thanos;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "ThnaosNFSv1", group = "NFS")
public class NFS extends MasqLinearOpMode {
    private Thanos thanos = new Thanos();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        thanos.mapHardware(hardwareMap);
        thanos.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            thanos.NFS(controller1);
            dash.create(thanos.tracker.getPosition());
            thanos.tracker.updateSystem();
            dash.update();
        }
    }
}

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
    private double adjusterPosition = 0;
    private double adjusterMax = 1;
    private double adjusterMin = 0;
    private double adjusterIncrement = 0.05;
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
            if (controller1.rightStickY() > 0) adjusterPosition += adjusterIncrement;
            else if (controller1.rightStickY() < 0) adjusterPosition -= adjusterIncrement;
            if (adjusterPosition > adjusterMax) adjusterPosition = adjusterMax;
            else if (adjusterPosition < adjusterMin) adjusterPosition = adjusterMin;
            thanos.adjuster.setPosition(adjusterPosition);
            thanos.lift.setPower(controller2.leftStickY());
            if (controller2.leftBumper()) thanos.collector.setPower(1);
            else if (controller2.leftBumper()) thanos.collector.setPower(-1);
            else thanos.collector.setPower(0);
            dash.create(thanos.tracker.getPosition());
            thanos.update();
        }
    }
}

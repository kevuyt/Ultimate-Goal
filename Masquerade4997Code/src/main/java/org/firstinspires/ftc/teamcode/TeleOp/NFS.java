package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "ThnaosNFSv1", group = "NFS")
public class NFS extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double adjusterPosition = 0;
    private double adjusterMax = 1;
    private double adjusterMin = 0;
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
            if (controller2.rightStickY() > 0) adjusterPosition += adjusterIncrement;
            else if (controller2.rightStickY() < 0) adjusterPosition -= adjusterIncrement;

            if (adjusterPosition > adjusterMax) adjusterPosition = adjusterMax;
            else if (adjusterPosition < adjusterMin) adjusterPosition = adjusterMin;

            if (controller2.leftBumper()) falcon.collector.setPower(.5);
            else if (controller2.rightBumper()) falcon.collector.setPower(-.5);
            else falcon.collector.setPower(0);

            falcon.rotator.DriverControl(controller1);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());
            //falcon.adjuster.setPosition(adjusterPosition);
            falcon.lift.setPower(controller2.leftStickY());
            dash.create(falcon.tracker.getPosition());
            //falcon.update();
        }
    }
}

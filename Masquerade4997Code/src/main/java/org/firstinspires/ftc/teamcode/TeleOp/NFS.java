package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "NFS", group = "NFS")
public class NFS extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
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
            if (controller2.rightStickY() > 0) adjusterPosition = adjusterMax;
            else if (controller2.rightStickY() < 0) adjusterPosition = adjusterMin;

            if (controller2.a()) falcon.dumper.setPosition(dumperMax);
            if (controller2.b()) falcon.dumper.setPosition(dumperMin);

            if (controller1.leftBumper()) falcon.collector.setPower(.5);
            else if (controller1.rightBumper()) falcon.collector.setPower(-.5);
            else falcon.collector.setPower(0);

            falcon.lift.setPower(controller2.leftStickY());
            falcon.adjuster.setPosition(adjusterPosition);
            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());
            dash.create(falcon.tracker.getPosition());
            dash.create(falcon.lift.getCurrentPosition());
            dash.create(falcon.rotator.getBasePower());
            dash.update();
        }
    }
}

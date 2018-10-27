package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 9/15/18.
 * Project: MasqLib
 */
@TeleOp(name = "NFS", group = "NFS")
public class NFS extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private double adjusterPosition = 0;
    private double adjusterMax = 1;
    private double adjusterMin = 0;
    private double dumperMin = 0.84;
    private double dumperMax = 0.2;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.endHang.setPosition(END_HANG_OUT);
        falcon.dumper.setPosition(dumperMin);
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

            if (controller1.y()) falcon.endSpool.setPower(1);
            else falcon.endSpool.setPower(0);

            if (controller1.b()) falcon.endHang.setPosition(END_HANG_IN);
            else if (controller1.x()) falcon.endHang.setPosition(END_HANG_OUT);
            //Set Power
            falcon.lift.setPower(controller2.leftStickY());
            falcon.adjuster.setPosition(adjusterPosition);
            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());

            //Dash
            dash.create(falcon.tracker.getPosition());
            dash.create(falcon.lift.getCurrentPosition());
            dash.create(falcon.rotator.getBasePower());
            dash.create(falcon.rotator.getRawPower());
            dash.update();
        }
    }
}

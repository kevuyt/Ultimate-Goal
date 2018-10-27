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
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.endHang.setPosition(END_HANG_OUT);
        falcon.dumper.setPosition(DUMPER_IN);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            falcon.NFS(controller1);
            if (controller2.rightStickY() > 0) adjusterPosition = ADJUSTER_IN;
            else if (controller2.rightStickY() < 0) adjusterPosition = ADJUSTER_OUT;

            if (controller2.a()) falcon.dumper.setPosition(DUMPER_IN);
            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);

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
            dash.create(controller1.leftBumper());
            dash.update();
        }
    }
}

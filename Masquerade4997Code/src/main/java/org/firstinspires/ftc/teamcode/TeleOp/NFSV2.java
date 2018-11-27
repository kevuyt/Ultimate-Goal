package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/27/18.
 * Project: MasqLib
 */
@TeleOp(name = "NFS", group = "Tank")
public class NFSV2 extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private boolean mode = true;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        //falcon.dogeForia.stop();
        falcon.initializeTeleop();
        falcon.hangSystem.setClosedLoop(true);
        falcon.driveTrain.setClosedLoop(true);
        falcon.dumper.setPosition(DUMPER_IN);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            falcon.NFS(controller1);
            if (controller1.a()) falcon.adjuster.setPosition(ADJUSTER_OUT);
            else if (controller1.y()) falcon.adjuster.setPosition(ADJUSTER_IN);

            if (controller1.leftBumper()) falcon.collector.setPower(.5);
            else if (controller1.leftTriggerPressed()) falcon.collector.setPower(-.5);
            else falcon.collector.setPower(0);

            if (controller2.b()) falcon.dumper.setPosition(DUMPER_OUT);
            else falcon.dumper.setPosition(DUMPER_IN);

            if (controller2.leftStickY() > 0) falcon.hangSystem.setPower(-1);
            else if (controller2.leftStickY() < 0) falcon.hangSystem.setPower(1);
            else falcon.hangSystem.setPower(0);
            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());

            falcon.lift.DriverControl(controller1);

            //Dash
            dash.create("Hang One", falcon.hangSystem.motor1.getPower());
            dash.create("Hang Two", falcon.hangSystem.motor2.getPower());
            dash.update();
            controller1.update();
        }
    }
}

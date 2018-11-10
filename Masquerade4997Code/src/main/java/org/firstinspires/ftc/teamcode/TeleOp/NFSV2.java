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
        falcon.goldAlignDetector.disable();
        falcon.initializeTeleop();
        falcon.driveTrain.setClosedLoop(true);
        falcon.endHang.setPosition(END_HANG_OUT);
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

            if (controller2.a()) falcon.endSpool.setPower(1);
            if (controller2.dPadDown()) falcon.endSpool.setPower(-1);
            else falcon.endSpool.setPower(0);

            if (controller1.dPadUp()) {
                falcon.endHang.setPosition(END_HANG_IN);
                falcon.driveTrain.setPower(0.3);
                sleep(1);
                falcon.driveTrain.setPower(0);
                falcon.endHang.setPosition(END_HANG_OUT);
                falcon.endSpool.setPower(1);
                sleep(10);
            }

            if (controller2.x()) falcon.endHang.setPosition(END_HANG_OUT);
            else falcon.endHang.setPosition(END_HANG_IN);

            falcon.rotator.DriverControl(controller2);
            falcon.rotator.setLiftPosition(falcon.lift.getCurrentPosition());

            falcon.lift.DriverControl(controller1);

            //Dash
            dash.create("Lift Position: ", falcon.lift.getCurrentPosition());
            dash.create("Rotator Position: ", falcon.rotator.getPosition());
            dash.create("KP: ", falcon.rotator.output.getKp());
            dash.update();
            controller1.update();
        }
    }
}

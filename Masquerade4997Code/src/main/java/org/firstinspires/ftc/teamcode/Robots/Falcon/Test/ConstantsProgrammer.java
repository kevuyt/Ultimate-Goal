package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@TeleOp(name = "ConstantsProgrammer", group = "Tank")

public class ConstantsProgrammer extends MasqLinearOpMode {
    private Falcon falcon = new Falcon();
    private double hangLatch = 0, adjuster = 0, endHang = 0;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("HELLO ");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.aOnPress()) {
                hangLatch += 0.01;
            }
            if (controller1.bOnPress()) {
                hangLatch -= 0.01;
            }
            if (controller1.xOnPress()) {
                adjuster += 0.01;
            }
            if (controller1.yOnPress()) {
                adjuster -= 0.01;
            }
            if (controller1.rightTriggerOnPress()) {
                endHang += 0.01;
            }
            if (controller1.leftTriggerOnPress()) {
                endHang -= 0.01;
            }
            controller1.update();
            falcon.markerDump.setPosition(endHang);
            //falcon.rotator.DriverControl(controller1);
            dash.create("Adjuster (RT+,LT-)", falcon.markerDump.getPosition());
            dash.create("Lift Position: ", falcon.lift.getCurrentPosition());
            dash.update();
        }

    }
}

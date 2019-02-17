package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/9/18.
 * Project: MasqLib
 */
@Autonomous(name = "BringLiftDown", group = "Autonomus")
public class BringLiftDown extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();

    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        while (!falcon.limitTop.isPressed() && opModeIsActive())
            falcon.hang.setVelocity(HANG_DOWN);
        falcon.hang.setPower(0);
    }
}
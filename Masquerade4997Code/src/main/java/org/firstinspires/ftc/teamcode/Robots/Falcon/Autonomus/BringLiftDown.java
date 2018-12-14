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
        falcon.hangSystem.motor1.enableStallDetection();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        while (!falcon.limitBottom.isPressed() && opModeIsActive())
            falcon.hangSystem.setVelocity(HANG_DOWN);
        falcon.hangSystem.setPower(0);
    }
}
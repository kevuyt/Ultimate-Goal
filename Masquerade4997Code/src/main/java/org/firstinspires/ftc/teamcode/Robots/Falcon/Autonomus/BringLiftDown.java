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
    Falcon boring = new Falcon();

    public void runLinearOpMode() throws InterruptedException {
        boring.mapHardware(hardwareMap);
        boring.hang.motor1.enableStallDetection();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(boring.imu);
            dash.update();
        }
        waitForStart();
        while (!boring.limitTop.isPressed() && opModeIsActive())
            boring.hang.setVelocity(HANG_DOWN);
        boring.hang.setPower(0);
    }
}
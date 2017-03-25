package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqLinearOpMode;

/**
 * A Template to follow for all TeleOp Opmodes
 */

@TeleOp(name = "Example-Teleop", group = "Example")
public class ExampleTeleOp extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            telemetry.addLine("Status");
            telemetry.update();
        }


    }
    public void waitNow(long waitTime) {
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

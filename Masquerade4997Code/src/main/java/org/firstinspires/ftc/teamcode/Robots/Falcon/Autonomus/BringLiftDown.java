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
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        while (!opModeIsActive()) {

            dash.update();
        }
        waitForStart();
        falcon.hang.unBreakMode();
        falcon.hang.setPower(0);
    }
}
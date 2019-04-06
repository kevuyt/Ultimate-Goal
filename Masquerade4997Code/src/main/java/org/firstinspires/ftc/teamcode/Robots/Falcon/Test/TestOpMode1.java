package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "TestOpMode1", group = "T")
public class TestOpMode1 extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            dash.update();
        }
        waitForStart();
        falcon.tracker.reset();
        falcon.gotoXY(22, 0, 0, 0.7);
        falcon.gotoXY(14, 0, 0, 0.7);
        falcon.gotoXY(15, 32, 45, 0.7);
        falcon.gotoXY(49, 63, 45, 0.7);
    }
}

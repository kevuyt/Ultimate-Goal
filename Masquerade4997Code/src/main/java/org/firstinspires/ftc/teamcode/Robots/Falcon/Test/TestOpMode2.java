package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "TestOpMode2", group = "T")
public class TestOpMode2 extends MasqLinearOpMode implements Constants {
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
        //falcon.gotoXY(0, 30, 90, 0.7);
        falcon.gotoXY(30, 30, 90, 0.7);
    }
}

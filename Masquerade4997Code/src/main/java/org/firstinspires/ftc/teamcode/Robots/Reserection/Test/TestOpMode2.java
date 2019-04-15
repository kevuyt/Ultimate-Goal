package org.firstinspires.ftc.teamcode.Robots.Reserection.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "TestOpMode2", group = "T")
@Disabled
public class TestOpMode2 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    public void runLinearOpMode() throws InterruptedException {
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.update();
        }
        waitForStart();
        resurrection.tracker.reset();
        resurrection.gotoXY(0, -50, -5);
    }
}

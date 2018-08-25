package org.firstinspires.ftc.teamcode.Test;

/**
 * Created by Archishmaan Peyyety on 8/25/18.
 * Project: MasqLib
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGoTest", group = "T")
public class MasqGoTest extends MasqLinearOpMode implements Constants {
    private Creed creed = new Creed();
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(creed.tracker.getGlobalX());
            dash.update();
        }
        waitForStart();
        creed.go(20, 20);
    }
}
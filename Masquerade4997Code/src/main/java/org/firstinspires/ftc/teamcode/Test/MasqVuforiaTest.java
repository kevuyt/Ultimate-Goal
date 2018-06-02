package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Creed;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "T")
public class MasqVuforiaTest extends MasqLinearOpMode {
    private Creed creed = new Creed();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        creed.mapHardware(hardwareMap);
        creed.vuforia.initVuforia(hardwareMap);
        dash.create(">>> Press Play");
        dash.update();
        waitForStart();
        creed.vuforia.activateVuMark();
        while (opModeIsActive()) {
            dash.create(creed.vuforia.getVuMark());
            dash.update();
        }
    }
}

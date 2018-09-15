package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Thanos;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "T")
public class MasqVuforiaTest extends MasqLinearOpMode {
    private Thanos thanos = new Thanos();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        thanos.mapHardware(hardwareMap);
        thanos.vuforia.initVuforia(hardwareMap);
        dash.create(">>> Press Play");
        dash.update();
        waitForStart();
        thanos.vuforia.activateVuMark();
        while (opModeIsActive()) {
            dash.create(thanos.vuforia.getVuMark());
            dash.update();
        }
    }
}

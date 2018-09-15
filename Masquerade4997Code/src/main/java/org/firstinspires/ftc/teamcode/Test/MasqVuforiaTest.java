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

    }
}

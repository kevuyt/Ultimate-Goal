package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "T")
public class MasqVuforiaTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        dash.create(">>> Press Play");
        dash.update();
        waitForStart();
        robot.vuforia.activateVuMark();
        while (opModeIsActive()) {
            dash.create(robot.vuforia.getVuMark());
            dash.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "Auto")
public class MasqVuforiaTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.vuforia.init();
        while (!opModeIsActive()){
            dash.create(robot.getTrackable());
            dash.create(robot.anInt());
            dash.create(robot.vuforia.vuMark.toString());
            dash.update();
        }
        waitForStart();
    }
}

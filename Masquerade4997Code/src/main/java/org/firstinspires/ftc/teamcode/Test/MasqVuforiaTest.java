package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "Auto")
public class MasqVuforiaTest extends MasqLinearOpMode {
    @Override
    public void run() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.init();
        while (!opModeIsActive()){
            dash.create(robot.vuforia.getTrackable());
        }
        waitForStart();
        while (opModeIsActive()) {

        }
    }
}

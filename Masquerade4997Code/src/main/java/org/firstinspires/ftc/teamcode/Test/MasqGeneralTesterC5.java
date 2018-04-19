package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 4/16/18.
 */
@Autonomous(name = "MasqGeneralTesterC5: TeleOp Toggle, Blue Jewel, Continuous t update", group = "T")
@Disabled
public class MasqGeneralTesterC5 extends MasqLinearOpMode {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        dash.startUpdate();
        int count = 0;
        while (!opModeIsActive()) {
            dash.create("Count: ", count);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.relicAdjuster.setPosition(Math.abs(controller1.leftStickX()));
        }
    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/7/17.
 */
@Autonomous(name = "RunUntil", group = "Autonomus")
public class RightForward extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        dash.create(">>> Press Play");
        dash.update();
        waitForStart();
        robot.vuforia.activateVuMark();
        robot.waitForVuMark();
        robot.drive(90);
    }
}
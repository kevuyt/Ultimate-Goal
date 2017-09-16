package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqRobot.Targets;
import Library4997.MasqSensors.MasqVuforia.Facing;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 9/7/17.
 */
@Autonomous(name = "TEST: VUFORIA", group = "Auto")
public class MasqVuforiaTest extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.vuforiaInit();
        while (!opModeIsActive()){
            dash.create("IS SEEN", robot.vuforia.isSeen(Targets.TARGET_ONE));
            dash.create("POSITION", robot.vuforia.position(Targets.TARGET_ONE));
            dash.update();
        }
    }
}

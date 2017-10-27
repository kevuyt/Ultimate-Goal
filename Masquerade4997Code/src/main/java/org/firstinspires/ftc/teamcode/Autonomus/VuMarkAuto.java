package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/6/17.
 */
@Autonomous(name = "VuMarkAuto", group = "Autonomus")
public class VuMarkAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        //robot.vuforia.init();
        while (opModeIsActive()) {
            dash.create(robot.vuforia);
        }
        waitForStart();
        MasqExternal.sleep(robot.getDelay());
        robot.drive(100);
        MasqExternal.sleep(100);
        robot.drive(100);
    }
}
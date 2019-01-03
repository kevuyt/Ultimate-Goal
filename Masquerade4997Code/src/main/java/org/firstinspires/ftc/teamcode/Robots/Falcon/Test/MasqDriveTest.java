package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

/**
 * Created by Archishmaan Peyyety on 8/25/18.
 * Project: MasqLib
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqDriveTesting", group = "T")
@Disabled
public class MasqDriveTest extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        double x = 0 , y = 0;
        while (!opModeIsActive()) {
            dash.create("X: " + x);
            dash.create("Y: " + y);
            if (controller1.xOnPress()) x = x + 0.1;
            if (controller2.yOnPress()) y = y + 0.1;
            controller1.update();
            dash.update();
            sleep(100f);
        }
        waitForStart();
        falcon.lift.runToPosition(Direction.BACKWARD, 2000);
    }
}
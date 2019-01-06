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
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        while (!opModeIsActive()) {
            dash.create("G");
            dash.update();
        }
        waitForStart();
        falcon.turnAbsolute(90, Direction.LEFT);
        sleep(1);
        falcon.turnAbsolute(0, Direction.RIGHT);
        sleep(1);
        falcon.turnAbsolute(90, Direction.LEFT);
        sleep(1);
        falcon.turnAbsolute(0, Direction.RIGHT);
        sleep(1);
        falcon.turnAbsolute(150, Direction.LEFT);
        sleep(1);
        falcon.turnAbsolute(0, Direction.RIGHT);
        sleep(1);
        falcon.turnAbsolute(150, Direction.LEFT);
        sleep(1);
        falcon.turnAbsolute(0, Direction.RIGHT);
        sleep(1);
        falcon.turnProportional(30, Direction.LEFT);
        sleep(1);
        falcon.turnProportional(30, Direction.RIGHT);
        sleep(1);
        falcon.turnProportional(30, Direction.LEFT);
        sleep(1);
        falcon.turnProportional(30, Direction.RIGHT);
    }
}
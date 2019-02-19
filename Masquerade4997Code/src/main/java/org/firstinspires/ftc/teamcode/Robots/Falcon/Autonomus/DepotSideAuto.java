
package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@Autonomous(name = "DepotSideAuto", group = "NFS")
public class DepotSideAuto extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        while (!opModeIsActive()) {
            dash.create(falcon.goldAlignDetector.getXPosition());
            dash.create(falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition()).toString());
            dash.create("LIFT: ", falcon.lift.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        //BlockPlacement blockPlacement = BlockPlacement.RIGHT;
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        dropMarker();
    }
    public void travelCenter() {
        falcon.strafe(-90, 20);
        falcon.strafe(90, 10);
    }
    public void travelLeft() {
        falcon.strafe(-70, 22);
        falcon.strafe(80, 11);
    }
    public void travelRight() {
        falcon.strafe(-130, 22);
        falcon.strafe(50, 15);
    }
    public void dropMarker() {
        falcon.lift.runToPosition(80, 1);
        falcon.collector.setPower(.5);
        sleep();
        falcon.collector.setPower(-.5);
        falcon.lift.runToPosition(-80, 1);
    }
    @Override
    public void stopLinearOpMode() {
        falcon.goldAlignDetector.disable();
    }
}
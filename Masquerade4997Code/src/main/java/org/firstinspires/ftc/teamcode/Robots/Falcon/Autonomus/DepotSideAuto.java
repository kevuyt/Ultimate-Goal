
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
@Autonomous(name = "DepotSideAuto", group = "Tank")
public class DepotSideAuto extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        while (!opModeIsActive()) {
            dash.create(falcon.goldAlignDetector.getXPosition());
            dash.create(falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition()).toString());
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.RIGHT) {
            falcon.turnAbsolute(50, Direction.RIGHT);
            falcon.drive(40, 0.8, Direction.FORWARD);
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(30);
            falcon.adjuster.setPosition(ADJUSTER_OUT);
            sleep(1);
            falcon.adjuster.setPosition(ADJUSTER_IN);
            falcon.turnAbsolute(50, Direction.RIGHT);
            falcon.drive(100, Direction.BACKWARD, 3);
        }
        else if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(50, 0.8, Direction.FORWARD);
            falcon.turnAbsolute(70, Direction.RIGHT);
            falcon.adjuster.setPosition(ADJUSTER_OUT);
            sleep(1);
            falcon.adjuster.setPosition(ADJUSTER_IN);
            falcon.drive(100, Direction.BACKWARD, 4);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(50, Direction.LEFT);
            falcon.drive(40, 0.8, Direction.FORWARD);
            falcon.turnAbsolute(50, Direction.RIGHT);
            falcon.drive(20);
            falcon.adjuster.setPosition(ADJUSTER_OUT);
            sleep(1);
            falcon.adjuster.setPosition(ADJUSTER_IN);
            falcon.drive(100, Direction.BACKWARD, 3);
        }
    }
    @Override
    public void stopLinearOpMode() {
        falcon.goldAlignDetector.disable();
    }
}
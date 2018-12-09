package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@Autonomous(name = "DepotSideAuto", group = "Tank")
public class DepotSideAuto extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    enum BlockPlacement {
        LEFT,
        RIGHT,
        CENTER,
    }
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.adjuster.setPosition(ADJUSTER_OUT);
        while (!opModeIsActive()) {
            dash.create("H");
            dash.update();
        }
        waitForStart();
        while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        BlockPlacement blockPlacement = getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.RIGHT) {
            falcon.turnAbsolute(40, Direction.RIGHT);
            falcon.drive(45, 0.8, Direction.FORWARD);
            falcon.turnRelative(90, Direction.LEFT, 3);
            falcon.drive(20);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.drive(80, Direction.BACKWARD, 3);
        }
        else if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(55, 0.8, Direction.FORWARD);
            falcon.turnAbsolute(55, Direction.RIGHT, 3);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.drive(100, Direction.BACKWARD, 4);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(40, 0.8, Direction.FORWARD);
            falcon.turnRelative(90, Direction.RIGHT, 3);
            falcon.drive(20);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.drive(80, Direction.BACKWARD, 3);
        }
    }
    @Override
    public void stopLinearOpMode() {
        falcon.goldAlignDetector.disable();
    }

    public BlockPlacement getBlockPlacement (int block) {
        MasqClock clock = new MasqClock();
        boolean seen = true;
        while (!clock.elapsedTime(1, MasqClock.Resolution.SECONDS) && falcon.goldAlignDetector.isFound()) {}
        if (clock.seconds() < 1) seen = false;
        if (!seen) return BlockPlacement.LEFT;
        else if (block < 200) return BlockPlacement.CENTER;
        else return BlockPlacement.RIGHT;
    }
}

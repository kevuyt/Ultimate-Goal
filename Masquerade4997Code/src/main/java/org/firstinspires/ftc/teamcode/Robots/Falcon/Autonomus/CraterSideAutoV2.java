package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 1/3/19.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSideAutoV2", group = "Autonomus")
public class CraterSideAutoV2 extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    enum BlockPlacement {
        LEFT,
        RIGHT,
        CENTER,
    }
    double x = 0, y = 0;
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.rotator.startHoldThread();
        while (!opModeIsActive()) {
            if (controller1.xOnPress()) x += 0.1;
            if (controller1.yOnPress()) y += 0.1;
            dash.update();
            controller1.update();
            dash.create("X: " + x);
            dash.create("Y: " + y);
        }
        waitForStart();
        BlockPlacement blockPlacement =
                getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        sleep(1);
        if (blockPlacement == BlockPlacement.CENTER) processCenter();
        falcon.dogeForia.stop();
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
    public void processRight () {
        falcon.turnAbsolute(-35, Direction.LEFT);
        falcon.drive(25);
        falcon.drive(8, Direction.BACKWARD);
        falcon.turnAbsolute(80, Direction.LEFT);
        falcon.drive(50);
        falcon.turnAbsolute(130, Direction.LEFT);
        falcon.drive(50);
        falcon.markerDump.setPosition(0);
        sleep(1);
        falcon.turnAbsolute(140, Direction.LEFT);
        falcon.drive(100, Direction.BACKWARD, 5);
    }
    public void processLeft () {
        falcon.turnAbsolute(40, Direction.LEFT);
        falcon.drive(25);
        falcon.drive(7, Direction.BACKWARD);
        falcon.turnAbsolute(80, Direction.LEFT);
        falcon.drive(45);
        falcon.turnAbsolute(130, Direction.LEFT);
        falcon.drive(50);
        falcon.markerDump.setPosition(0);
        sleep(1);
        falcon.turnAbsolute(140, Direction.LEFT);
        falcon.drive(100, Direction.BACKWARD, 5);
    }
    public void processCenter () {
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hangSystem.setPower(-1);
                while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setPower(1);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.BACKWARD, 2000);
                falcon.rotator.setAngle(26, Direction.BACKWARD);
                falcon.rotator.setMovementAllowed(true);
                sleep(1);
            }
        });
        falcon.collector.setPower(0.5);
        falcon.lift.runToPosition(Direction.BACKWARD, 4000);
        sleep(1);
        falcon.lift.runToPosition(Direction.FORWARD, 2000);
        falcon.drive(20);
        falcon.rotator.setAngle(0, Direction.FORWARD);
        falcon.turnAbsolute(falcon.getRotatorCoordinates(x, y)[0], Direction.LEFT);
        falcon.lift.runToPosition(Direction.BACKWARD, 5000);
        falcon.rotator.setMovementAllowed(true);
        falcon.collector.setPower(.5);
        falcon.rotator.setAngle(20, Direction.FORWARD);
        falcon.lift.runToPosition(Direction.FORWARD, 2000);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(20, Direction.BACKWARD);
        falcon.turnAbsolute(30, Direction.RIGHT);
        falcon.rotator.setAngle(80, Direction.FORWARD);
        falcon.rotator.setMovementAllowed(true);
        falcon.lift.runToPosition(Direction.BACKWARD, 4000);

    }
    @Override
    public void stopLinearOpMode () {
        falcon.rotator.close();
    }
}
package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
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
        /*BlockPlacement blockPlacement =
                getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        //if (blockPlacement == BlockPlacement.CENTER)*/
            processCenter();
        falcon.dogeForia.stop();
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
                //while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hangSystem.setPower(-1);
                //while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setPower(1);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.OUT, 2000);
                falcon.rotator.setAngle(20, Direction.BACKWARD);
                falcon.rotator.setMovementAllowed(true);
                falcon.collector.setPower(0.5);
                sleep(.5);
                falcon.lift.runToPosition(Direction.OUT, 4000);
                sleep(1);
            }
        });
        falcon.rotator.setAngle(0, Direction.UP);
        falcon.lift.runToPosition(Direction.IN, 3000);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(10);
                if (x < 1) falcon.turnRelative(30, Direction.RIGHT);
                if (x >= 1) falcon.turnRelative(30, Direction.LEFT);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.OUT, 4000);
            }
        });
        falcon.rotator.setAngle(26, Direction.DOWN);
        falcon.rotator.setMovementAllowed(true);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.shake(4, 5, .5);
                falcon.collector.setPower(0);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.OUT, 5000);
            }
        });
        falcon.rotator.setAngle(60, Direction.UP);
        falcon.lift.runToPosition(Direction.IN, 3000);
        if (falcon.tracker.getHeading() < 0)
            falcon.turnAbsolute(0, Direction.LEFT);
        else falcon.turnAbsolute(0, Direction.RIGHT);
    }

    @Override
    public void stopLinearOpMode () {
        falcon.rotator.close();
    }
}
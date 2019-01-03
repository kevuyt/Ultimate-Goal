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
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.hangSystem.motor1.enableStallDetection();
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
        falcon.hangSystem.setPower(0);
        sleep(1);
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.CENTER) processCenter();
        /*else if (blockPlacement == BlockPlacement.LEFT) processLeft();
        else processRight();*/
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
                while (!falcon.limitBottom.isPressed()) falcon.hangSystem.setPower(-1);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.collector.setPower(.5);
                falcon.lift.runToPosition(Direction.BACKWARD, 1000);
                sleep(1);
            }
        });
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(5);
                while (!falcon.limitTop.isPressed()) falcon.hangSystem.setPower(1);
                falcon.drive(5, Direction.BACKWARD);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.rotator.setPower(Direction.FORWARD, 1);
                falcon.lift.runToPosition(Direction.BACKWARD, 1000);
                falcon.turnAbsolute(-30, Direction.LEFT);
                falcon.dumper.setPosition(DUMPER_OUT);
                sleep(2);
            }
        });
    }
}
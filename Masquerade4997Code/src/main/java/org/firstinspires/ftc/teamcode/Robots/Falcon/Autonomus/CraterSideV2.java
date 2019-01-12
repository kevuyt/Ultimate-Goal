package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 1/4/19.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSideAutoV2", group = "Autonomus")
public class CraterSideV2 extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.dumper.setPosition(DUMPER_IN);
        falcon.rotator.startHoldThread();
        while (!opModeIsActive()) {
            dash.create("Hola");
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement =
                falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        if (blockPlacement == BlockPlacement.CENTER) grabSampleOne(false);
        else if (blockPlacement == BlockPlacement.RIGHT) processRight();
        else processLeft();

        travelSampleTwo(blockPlacement);
        grabSampleTwo();
        falcon.dogeForia.stop();
    }

    public void processRight() {
        falcon.drive(2);
        falcon.turnAbsolute(35, Direction.RIGHT);
        grabSampleOne(false);
        falcon.turnAbsolute(0, Direction.LEFT);
    }
    public void processLeft() {
        falcon.turnAbsolute(35, Direction.LEFT);
        falcon.rotator.setAngle(0, Direction.UP);
        grabSampleOne(true, 20);
        falcon.turnAbsolute(0, Direction.RIGHT);
    }

    public void grabSampleOne(boolean correction, final double angle) {
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
        if (correction) falcon.turnAbsolute(0, Direction.RIGHT);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.rotator.setAngle(80, Direction.UP);
                if (Math.abs(falcon.lift.getCurrentPosition()) < 4000)
                falcon.lift.runToPosition(Direction.OUT, 4000);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.drive(2, Direction.BACKWARD);
                falcon.turnAbsolute(angle, Direction.RIGHT); //30
            }
        });
        falcon.lift.runToPosition(Direction.OUT, 5000);
        falcon.dumper.setPosition(DUMPER_OUT);
        sleep(1);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.rotator.setAngle(0, Direction.DOWN);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.IN, 1000);
            }
        });
    }
    public void grabSampleOne(boolean correction) {grabSampleOne(correction, 30);}

    public void travelSampleTwo(BlockPlacement placement) {
        falcon.collector.setPower(0);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(10);
        falcon.turnAbsolute(90, Direction.LEFT);
        if (placement == BlockPlacement.LEFT) {
            driveToWall(10, 4);
            falcon.turnAbsolute(135, Direction.LEFT);
            driveToWall(10, 3);
            falcon.turnAbsolute(-90, Direction.LEFT, 5);
        }
        if (placement == BlockPlacement.CENTER) {
            driveToWall(10, 4);
            falcon.turnAbsolute(135, Direction.LEFT);
            driveToWall(21, 3);
            falcon.turnRelative(90, Direction.LEFT);
        }
    }
    public void grabSampleTwo() {
        falcon.collector.setPower(0.5);
        falcon.lift.runToPosition(Direction.OUT, 1000);
        falcon.rotator.setAngle(26, Direction.DOWN);
        falcon.rotator.setMovementAllowed(true);
        falcon.lift.runToPosition(Direction.OUT, 4000);
        falcon.collector.setPower(0);
    }

    public void driveToWall (final double distance, int timeout) {
        falcon.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return falcon.distance.distance(DistanceUnit.INCH) > distance;
            }
        }, timeout);
    }
    @Override
    public void stopLinearOpMode() {
        falcon.rotator.close();
    }

}

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
@Autonomous(name = "ScoreAuto", group = "Autonomus")
public class ScoreAuto extends MasqLinearOpMode implements Constants {
    int rotatorDown = 15;
    int leftRightTurn = 40;
    int position = 0;
    int rotation = 0;
    Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.dumper.setPosition(DUMPER_IN);
        falcon.rotator.startHoldThread();
        while (!opModeIsActive()) {
            if (controller1.aOnPress()) position++;
            if (position > 2) position = 0;
            dash.create(position);
            controller1.update();
            dash.update();
        }
        waitForStart();
        falcon.pidPackage().setKpTurn(0.03);
        setRotation();
        final BlockPlacement blockPlacement =
                falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        unHang();
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(5);
                if (blockPlacement == BlockPlacement.RIGHT) {
                    falcon.turnAbsolute(-leftRightTurn, Direction.LEFT);
                    grabSampleOne();
                    falcon.turnAbsolute(0, Direction.LEFT);
                }
                else if (blockPlacement == BlockPlacement.LEFT) {
                    falcon.turnAbsolute(leftRightTurn, Direction.LEFT);
                    grabSampleOne();
                    falcon.turnAbsolute(0, Direction.RIGHT);
                }
                else grabSampleOne();
            }
        }, new Runnable() {
            @Override
            public void run() {
                while (!falcon.limitTop.isPressed() && opModeIsActive())
                    falcon.hangSystem.setVelocity(HANG_DOWN);
                falcon.hangSystem.setPower(0);
            }
        });

        falcon.turnAbsolute(40, Direction.RIGHT);
        falcon.drive(5, Direction.BACKWARD);
        falcon.rotator.setAngle(0, Direction.UP);
        falcon.rotator.setAngle(60, Direction.UP);
        falcon.lift.runToPosition(Direction.OUT, 4000);
        falcon.dumper.setPosition(DUMPER_OUT);
        sleep();
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.rotator.setAngle(40, Direction.DOWN);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.IN, 1000);
            }
        });

        falcon.dogeForia.stop();
    }


    public void driveToWall (final double distance, int timeout) {
        falcon.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return falcon.distance.distance(DistanceUnit.INCH) > distance;
            }
        }, timeout);
    }
    public void unHang() {
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.OUT, 2000);
            }
        });
    }
    public void grabSampleOne() {
        falcon.rotator.setAngle(rotatorDown, Direction.DOWN);
        falcon.collector.setPower(.5);
        falcon.lift.runToPosition(Direction.OUT, 4000);
        sleep(1.5);
        falcon.lift.runToPosition(Direction.IN, 2000);
        falcon.collector.setPower(0);
    }
    public void grabSampleTwo() {
        falcon.collector.setPower(.5);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(5);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.rotator.setAngle(0, Direction.UP, 1);
                falcon.rotator.setAngle(5, Direction.UP, 1);
                falcon.lift.runToPosition(Direction.OUT, 5000);
            }
        });
        falcon.rotator.setAngle(rotatorDown, Direction.DOWN, 1);
        falcon.drive(7);
        sleep(2);
        falcon.drive(7, Direction.BACKWARD);
    }
    @Override
    public void stopLinearOpMode() {
        falcon.rotator.close();
    }
    public void setRotation () {
        if (position == 0) rotation = leftRightTurn;
        else if (position == 2) rotation = -leftRightTurn;
    }

}

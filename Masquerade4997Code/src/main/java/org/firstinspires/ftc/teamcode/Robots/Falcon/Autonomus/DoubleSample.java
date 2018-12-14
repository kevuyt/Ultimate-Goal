package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/1/18.
 * Project: MasqLib
 */
@Autonomous(name = "DoubleSample", group = "Autonomus")
public class DoubleSample extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    enum BlockPlacement {LEFT, RIGHT, CENTER}
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.hangSystem.motor1.enableStallDetection();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        BlockPlacement blockPlacement = getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(20);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(137, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-90, Direction.LEFT, 5);
            falcon.drive(30);
            falcon.drive(30, Direction.BACKWARD);
            falcon.turnAbsolute(132, Direction.LEFT, 3);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(25);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(135, Direction.LEFT);
            driveToWall(10);
            falcon.markerDump.setPosition(0);
            falcon.sleep(1);
            // Decrease is more inward.
            falcon.turnAbsolute(-120, Direction.LEFT, 5);
            falcon.drive(40);
            falcon.drive(40, Direction.BACKWARD);
            falcon.turnAbsolute(132, Direction.LEFT);
        }
        else {
            falcon.turnAbsolute(40, Direction.RIGHT);
            falcon.drive(25);
            falcon.drive(10, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(150, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(90, Direction.LEFT);
            falcon.markerDump.setPosition(0);
            falcon.sleep(1);
            falcon.turnAbsolute(145, Direction.LEFT);
            // Decrease is more inward.
        }
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(100, Direction.BACKWARD, 5);

            }
        }, new Runnable() {
            @Override
            public void run() {
                while (!falcon.limitBottom.isPressed() && opModeIsActive())
                    falcon.hangSystem.setVelocity(HANG_DOWN);
                falcon.hangSystem.setPower(0);
            }
        });
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
    public void driveToWall (final double distance) {
        falcon.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return falcon.distance.distance(DistanceUnit.INCH) > distance;
            }
        }, 15);
    }
}
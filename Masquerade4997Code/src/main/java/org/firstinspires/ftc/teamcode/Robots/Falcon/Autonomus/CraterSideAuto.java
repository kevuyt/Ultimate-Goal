package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqSensors.MasqClock;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/10/18.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSideAuto", group = "Autonomus")
public class CraterSideAuto extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    private int wallTurn = 130;
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        sleep(1);
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(25);
            falcon.drive(7, Direction.BACKWARD);

        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(7, Direction.BACKWARD);
        }
        else {
            falcon.turnAbsolute(-40, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(7, Direction.BACKWARD);
        }
        falcon.turnAbsolute(70, Direction.LEFT);
        falcon.drive(30);
        driveToWall(10);
        falcon.turnAbsolute(wallTurn, Direction.LEFT);
        falcon.drive(45);
        falcon.turnAbsolute(145, Direction.LEFT);
        falcon.markerDump.setPosition(0);
        sleep(1);
        final MasqClock clock = new MasqClock();
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(100, Direction.BACKWARD, 5);
            }
        }, new Runnable() {
            @Override
            public void run() {
                while (!falcon.limitTop.isPressed() && opModeIsActive() &&
                        !clock.elapsedTime(4, MasqClock.Resolution.SECONDS))
                    falcon.hangSystem.setVelocity(HANG_DOWN);
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
    public void driveToWall(final double di) {
        driveToWall(di, 5);
    }

    @Override
    public void stopLinearOpMode() {
        falcon.dogeForia.stop();
    }
}

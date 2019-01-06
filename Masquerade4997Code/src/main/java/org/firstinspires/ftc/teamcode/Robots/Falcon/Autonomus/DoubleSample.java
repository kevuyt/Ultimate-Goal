package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/1/18.
 * Project: MasqLib
 */
@Autonomous(name = "DoubleSample", group = "Autonomus")
public class DoubleSample extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        falcon.hangSystem.motor1.enableStallDetection();
        falcon.rotator.startHoldThread();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(20);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10, 4);
            falcon.turnAbsolute(140, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-90, Direction.LEFT, 5);
            falcon.drive(30);
            falcon.drive(30, Direction.BACKWARD);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.drive(3);
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(25);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10, 4);
            falcon.turnAbsolute(140, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-120, Direction.LEFT, 5);
            falcon.drive(40);
            falcon.drive(40, Direction.BACKWARD);
        }
        else {
            falcon.drive(3);
            falcon.turnAbsolute(40, Direction.RIGHT);
            falcon.drive(25);
            falcon.drive(10, Direction.BACKWARD);
            falcon.turnAbsolute(90, Direction.LEFT);
            driveToWall(10, 4);
            falcon.turnAbsolute(150, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(90, Direction.LEFT);
        }
        falcon.turnAbsolute(-30, Direction.LEFT, 3);
        falcon.markerDump.setPosition(0);
        falcon.sleep(1);

        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.drive(50,Direction.FORWARD,5);

            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.lift.runToPosition(Direction.OUT, 6000);
                falcon.rotator.setAngle(26, Direction.DOWN);
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
        falcon.rotator.close();
    }
}
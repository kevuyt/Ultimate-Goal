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
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        while (!falcon.limitTop.isPressed() && opModeIsActive()) falcon.hangSystem.setVelocity(HANG_UP);
        falcon.hangSystem.setPower(0);
        sleep(1);
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(23);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(80, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(130, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-100, Direction.LEFT);
            falcon.drive(30);
            falcon.drive(30, Direction.BACKWARD);
            falcon.turnAbsolute(-45, Direction.LEFT);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(30, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(80, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(130, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-130, Direction.LEFT);
            falcon.drive(30);
            falcon.drive(30, Direction.BACKWARD);
            falcon.turnAbsolute(-45, Direction.LEFT);
        }
        else {
            falcon.turnAbsolute(-30, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(80, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(140, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(-30, Direction.LEFT);
        }
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
}
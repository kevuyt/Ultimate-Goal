package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqHelpers.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/10/18.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSideAuto", group = "Autonomus")
public class CraterSideAuto extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        falcon.hangSystem.motor1.enableStallDetection();
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
            falcon.drive(45);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.turnAbsolute(140, Direction.LEFT);
            falcon.drive(100, Direction.BACKWARD, 5);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(30, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(80, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(130, Direction.LEFT);
            falcon.drive(50);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.turnAbsolute(140, Direction.LEFT);
            falcon.drive(100, Direction.BACKWARD, 5);
        }
        else {
            falcon.turnAbsolute(-30, Direction.LEFT);
            falcon.drive(28);
            falcon.drive(5, Direction.BACKWARD);
            falcon.turnAbsolute(80, Direction.LEFT);
            driveToWall(10);
            falcon.turnAbsolute(130, Direction.LEFT);
            falcon.drive(50);
            falcon.markerDump.setPosition(0);
            sleep(1);
            falcon.turnAbsolute(155, Direction.LEFT);
            falcon.drive(100, Direction.BACKWARD, 5);
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

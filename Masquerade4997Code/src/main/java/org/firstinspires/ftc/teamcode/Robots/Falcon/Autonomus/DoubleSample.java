package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    private int wallTurn = 130;
    private int sampleTurn;
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
        while (!falcon.limitBottom.isPressed() && opModeIsActive()) falcon.hang.setVelocity(HANG_UP);
        falcon.hang.setPower(0);
        sleep(1);
        falcon.drive(5);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.drive(20);
            falcon.drive(10, Direction.BACKWARD);
            sampleTurn = 70;
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(40, Direction.LEFT);
            falcon.drive(20);
            falcon.drive(10, Direction.BACKWARD);
            sampleTurn = 80;
            wallTurn = 125;
        }
        else {
            falcon.turnAbsolute(-40, Direction.LEFT);
            falcon.drive(20);
            falcon.drive(10, Direction.BACKWARD);
            sampleTurn = 75;
        }
        falcon.turnAbsolute(sampleTurn, Direction.LEFT);
        falcon.drive(30);
        driveToWall(10);
        if (blockPlacement == BlockPlacement.RIGHT) falcon.drive(7, Direction.BACKWARD);
        falcon.turnAbsolute(wallTurn, Direction.LEFT);
        driveToWall(10);
        if (blockPlacement == BlockPlacement.CENTER) {
            falcon.turnAbsolute(-100, Direction.LEFT);
            falcon.drive(30);
            falcon.drive(45, Direction.BACKWARD);
            falcon.turnAbsolute(-40, Direction.LEFT);
        }
        else if (blockPlacement == BlockPlacement.LEFT) {
            falcon.turnAbsolute(-130, Direction.LEFT);
            falcon.drive(30);
            falcon.drive(45, Direction.BACKWARD);
            falcon.turnAbsolute(-40, Direction.LEFT);
        }
        else {
            falcon.turnAbsolute(-30, Direction.LEFT);
        }
        falcon.drive(100, 0.7, Direction.FORWARD, 5);
        falcon.dogeForia.stop();
    }
    public void driveToWall (final double distance, int timeout) {
        falcon.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return false;// falcon.distance.distance(DistanceUnit.INCH) > distance;
            }
        }, timeout);
    }
    public void driveToWall(final double di) {
        driveToWall(di, 5);
    }
}
package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
    private int wallTurn = 130;
    private int sampleTurn;
    public void runLinearOpMode() {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();

        //BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        BlockPlacement blockPlacement = BlockPlacement.CENTER;
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();

        falcon.driveProportional(0,0.1);


        //falcon.goldAlignDetector.disable();
    }
    public void travelCenter() {
        falcon.strafe(-100, 25);
        falcon.strafe(90, 10);
        falcon.setEncoderPID(false);
        falcon.driveAbsoluteAngle(35, 0, 0.8);
        falcon.driveProportional(35,0.1);
        falcon.drive(20);
        falcon.drive(20, Direction.BACKWARD);
        falcon.driveProportional(0, 0.1);
        falcon.driveAbsoluteAngle(15, 0, 0.8, Direction.BACKWARD);
    }
    public void travelLeft() {
        falcon.strafe(-70, 40);
        falcon.setEncoderPID(false);
        falcon.driveProportional(40,0.3);
        falcon.drive(40);
        falcon.drive(35, Direction.BACKWARD);
        falcon.driveProportional(0, 0.1);
        falcon.driveAbsoluteAngle(15, 0, 0.8, Direction.BACKWARD);
    }
    public void travelRight() {
        falcon.strafe(-120, 25);
        falcon.strafe(70, 10);
        falcon.setEncoderPID(false);
        falcon.driveAbsoluteAngle(35, 0, 0.8);
        falcon.driveProportional(35,0.1);
        falcon.drive(20);
        falcon.drive(20, Direction.BACKWARD);
        falcon.driveProportional(0, 0.1);
        falcon.driveAbsoluteAngle(15, 0, 0.8, Direction.BACKWARD);
    }
    public void scoreSetup() {
        falcon.lift.setDistance(3000);
        falcon.lift.runToPosition(Direction.FORWARD, 1);
        falcon.rotator.rotator.setPower(1 * Direction.DOWN.value);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.lift.setDistance(3000);
                falcon.lift.runToPosition(Direction.FORWARD, 1);
            }
        }, new Runnable() {
            @Override
            public void run() {

            }
        });
    }

    public void driveToWall (final double distance, int timeout) {
        falcon.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return false; //falcon.distance.distance(DistanceUnit.INCH) > distance;
            }
        }, timeout);
    }
    public void driveToWall(final double di) {
        driveToWall(di, 5);
    }
}

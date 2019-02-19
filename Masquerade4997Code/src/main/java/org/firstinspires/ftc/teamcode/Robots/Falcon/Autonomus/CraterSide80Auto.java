package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 12/10/18.
 * Project: MasqLib
 */
@Autonomous(name = "CraterSide80Auto", group = "Autonomus")
public class CraterSide80Auto extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    BlockPlacement blockPlacement;
    public void runLinearOpMode() {
         falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        falcon.hang.setBreakMode();
        while (!opModeIsActive()) {
            dash.create(falcon.goldAlignDetector.getXPosition());
            dash.create(falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition()).toString());
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        grabBlock();
        falcon.rotator.rotator.setBreakMode();
        //unHang();
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();
        falcon.driveTrain.setVelocity(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
        falcon.drive(30, Direction.BACKWARD);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.lift.runToPosition(40, 1);
        //scoreSetup();
        //score();
        //falcon.goldAlignDetector.disable();
    }
    public void travelCenter() {
        falcon.strafe(-90, 20);
        falcon.strafe(90, 10);
        falcon.driveAbsoluteAngle(30, 0);
        falcon.driveProportional(45,0.1, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(15);
        falcon.drive(15, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelLeft() {
        falcon.strafe(-70, 20);
        falcon.driveProportional(40,0.2, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(35);
        falcon.drive(20, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelRight() {
        falcon.strafe(-120, 20);
        falcon.strafe(60, 7);
        falcon.driveAbsoluteAngle(35, 0, 0.8);
        falcon.driveProportional(45,0.1, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(20);
        falcon.drive(12, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void scoreSetup() {
        falcon.rotator.rotator.setPower(0);
        falcon.rotator.rotator.setBreakMode();
        falcon.driveAbsoluteAngle(20, 0, 0.8, Direction.BACKWARD);
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.turnAbsolute(90, Direction.RIGHT);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.collector.setPower(.5);
                falcon.lift.setDistance(2000);
                //falcon.lift.runToPosition(Direction.OUT, 1);
                falcon.rotator.rotator.unBreakMotors();
                falcon.lift.setDistance(2000);
                //falcon.lift.runToPosition(Direction.OUT, 1);
                falcon.lift.setBreakMode();
                falcon.rotator.rotator.setPower(0);
            }
        });
        sleep(2);
        falcon.collector.setPower(0);
    }
    public void score() {
        runSimultaneously(new Runnable() {
            @Override
            public void run() {
                falcon.rotator.rotator.setPower(Direction.UP.value * 1);
                sleep(2);
                falcon.rotator.rotator.setPower(0);
            }
        }, new Runnable() {
            @Override
            public void run() {
                falcon.drive(10, Direction.BACKWARD);
            }
        });
        falcon.dumper.setPosition(DUMPER_OUT);
        sleep(1);
    }

    public void unHang() {
        falcon.hang.unBreakMode();
        while (!falcon.limitTop.isPressed() && opModeIsActive())
            falcon.hang.setVelocity(HANG_DOWN);
        falcon.hang.setPower(0);
    }
    public void grabBlock() {
        blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
    }
}

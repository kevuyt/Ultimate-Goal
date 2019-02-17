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
@Autonomous(name = "CraterSideAuto", group = "Autonomus")
public class CraterSideAuto extends MasqLinearOpMode implements Constants {
    Falcon falcon = new Falcon();
    BlockPlacement blockPlacement;
    private int wallTurn = 130;
    private int sampleTurn;
    public void runLinearOpMode() {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.driveTrain.setClosedLoop(true);
        falcon.hang.setBreakMode();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.create(falcon.imu);
            dash.update();
        }
        waitForStart();
        //grabBlock();
        BlockPlacement blockPlacement = BlockPlacement.CENTER;
        unHang();
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();
        //scoreSetup();
        //score();
        //falcon.goldAlignDetector.disable();
    }
    public void travelCenter() {
        falcon.strafe(-90, 25);
        falcon.strafe(90, 10);
        falcon.setEncoderPID(false);
        falcon.driveAbsoluteAngle(35, 0, 0.8);
        falcon.driveProportional(35,0.1, Direction.LEFT);
        falcon.drive(20);
        falcon.drive(18, Direction.BACKWARD);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelLeft() {
        falcon.strafe(-100, 25);
        falcon.setEncoderPID(false);
        falcon.driveProportional(40,0.3, Direction.LEFT);
        falcon.drive(40);
        falcon.drive(35, Direction.BACKWARD);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelRight() {
        falcon.strafe(-120, 25);
        falcon.strafe(70, 10);
        falcon.setEncoderPID(false);
        falcon.driveAbsoluteAngle(35, 0, 0.8);
        falcon.driveProportional(35,0.1, Direction.LEFT);
        falcon.drive(20);
        falcon.drive(20, Direction.BACKWARD);
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
                falcon.lift.runToPosition(Direction.OUT, 1);
                falcon.rotator.rotator.unBreakMotors();
                falcon.lift.setDistance(2000);
                falcon.lift.runToPosition(Direction.OUT, 1);
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
        while (!falcon.limitBottom.isPressed() && opModeIsActive())
            falcon.hang.setVelocity(HANG_UP);
        falcon.hang.setPower(0);
    }
    public void grabBlock() {
        blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
    }
}

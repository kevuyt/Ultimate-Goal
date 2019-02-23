package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqSensors.MasqClock;
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
        falcon.hang.setBreakMode();
        falcon.tracker.imu.reset();
        falcon.hang.setPower(0);
        while (!opModeIsActive()) {
            dash.create(falcon.goldAlignDetector.getXPosition());
            dash.create(falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition()).toString());
            dash.update();
        }
        waitForStart();
        grabBlock();
        falcon.rotator.rotator.setBreakMode();
        unHang();
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();
        falcon.driveTrain.setVelocity(0);
        falcon.turnRelative(90, Direction.RIGHT);
        falcon.lift.runToPosition(80, 1);
        falcon.goldAlignDetector.disable();
    }
    public void travelCenter() {
        falcon.strafe(-95, 17, 0, 0.6);
        falcon.strafe(90, 5, 0, 0.6);
        falcon.driveAbsoluteAngle(30, 0);
        falcon.driveProportional(40,0.15, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(27);
        falcon.drive(17, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelLeft() {
        falcon.strafe(-90, 12, 0, 0.7);
        falcon.drive(8);
        falcon.strafe(-90, 8, 0, 0.7);
        falcon.driveProportional(40,0.1, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(37);
        falcon.drive(13, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }
    public void travelRight() {
        falcon.strafe(-90, 9, 0, 0.8);
        falcon.drive(15, Direction.BACKWARD);
        falcon.strafe(-90, 10, 0, 0.8);
        falcon.strafe(90, 8, 0, 0.8);
        falcon.driveAbsoluteAngle(50, 0, 0.8);
        falcon.driveProportional(40,0.1, Direction.LEFT);
        falcon.collector.setPower(-.5);
        falcon.drive(20);
        falcon.drive(15, Direction.BACKWARD);
        falcon.collector.setPower(0);
        falcon.driveProportional(0, 0.1, Direction.LEFT);
    }

    public void unHang() {
        MasqClock clock = new MasqClock();
        falcon.hang.unBreakMode();
        while (opModeIsActive() && !falcon.limitTop.isPressed() && !clock.elapsedTime(3, MasqClock.Resolution.SECONDS))
            falcon.hang.setPower(-1);
        falcon.hang.setPower(0);
    }
    public void grabBlock() {
        blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
    }
}

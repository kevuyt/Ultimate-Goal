
package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 10/20/18.
 * Project: MasqLib
 */
@Autonomous(name = "DepotParkWrongCrater", group = "NFS")
public class DepotParkWrongCrater extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    @Override
    public void runLinearOpMode() {
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.hang.setBreakMode();
        falcon.hang.setPower(0);
        while (!opModeIsActive()) {
            dash.create(falcon.goldAlignDetector.getXPosition());
            dash.create(falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition()).toString());
            dash.create("LIFT: ", falcon.lift.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        unHang();
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();

    }
    public void travelCenter() {
        falcon.strafe(-95, 25, 0, 0.6);
        falcon.strafe(90, 10, 0, 0.6);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.lift.runToPosition(-70, 1);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(35, Direction.BACKWARD);
        falcon.turnRelative(135, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
    }
    public void travelLeft() {
        falcon.strafe(-90, 10, 0, 0.7);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.lift.runToPosition(-70, 1);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(8);
        falcon.strafe(-90, 10, 0, 0.7);
        falcon.strafe(90, 8, 0, 0.7);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(45, Direction.BACKWARD);
        falcon.turnRelative(135, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
    }
    public void travelRight() {
        falcon.strafe(-90, 10, 0, 0.7);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.lift.runToPosition(-70, 1);
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(12, Direction.BACKWARD);
        falcon.strafe(-90, 13, 0, 0.7);
        falcon.strafe(90, 13, 0, 0.7);
        falcon.drive(15, Direction.BACKWARD);
        falcon.turnRelative(135, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
    }
    public void dropMarker() {
        falcon.lift.runToPosition(80, 1);
        falcon.collector.setPower(-.5);
        sleep(3);
        falcon.collector.setPower(0);
    }
    public void unHang() {
        falcon.hang.unBreakMode();
        while (opModeIsActive() && !falcon.limitTop.isPressed()) falcon.hang.setPower(-1);
        falcon.hang.setPower(0);
    }
    @Override
    public void stopLinearOpMode() {
        falcon.goldAlignDetector.disable();
    }
}
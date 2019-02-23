
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
@Autonomous(name = "DepotSideAuto", group = "NFS")
public class DepotSideAuto extends MasqLinearOpMode implements Constants {
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
            dash.create(falcon.driveTrain.leftDrive.motor1.getCurrentPosition());
            dash.create(falcon.driveTrain.leftDrive.motor2.getCurrentPosition());
            dash.create(falcon.driveTrain.rightDrive.motor1.getCurrentPosition());
            dash.create(falcon.driveTrain.rightDrive.motor2.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        unHang();
        BlockPlacement blockPlacement = falcon.getBlockPlacement((int) falcon.goldAlignDetector.getXPosition());
        //BlockPlacement blockPlacement = BlockPlacement.RIGHT;
        falcon.drive(2);
        if (blockPlacement == BlockPlacement.CENTER) travelCenter();
        else if (blockPlacement == BlockPlacement.LEFT) travelLeft();
        else travelRight();

    }
    public void travelCenter() {
        falcon.strafe(-90, 25, 0, 0.6);
        falcon.strafe(90, 10, 0, 0.6);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(25);
        falcon.turnAbsolute(40, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
    }
    public void travelLeft() {
        falcon.strafe(-90, 12, 0, 0.7);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(8);
        falcon.strafe(-90, 15, 0, 0.7);
        falcon.turnRelative(30, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
        falcon.drive(15);
    }
    public void travelRight() {
        falcon.strafe(-90, 12, 0, 0.7);
        falcon.turnAbsolute(90, Direction.RIGHT);
        falcon.rotator.rotator.setBreakMode();
        falcon.rotator.rotator.setPower(0);
        dropMarker();
        falcon.turnAbsolute(0, Direction.LEFT);
        falcon.drive(15, Direction.BACKWARD);
        falcon.strafe(-90, 13, 0, 0.7);
        falcon.strafe(90, 12, 0, 0.7);
        falcon.drive(43);
        falcon.turnRelative(30, Direction.LEFT);
        falcon.lift.runToPosition(100, 1);
    }
    public void dropMarker() {
        falcon.lift.runToPosition(100, 1);
        falcon.collector.setPower(-.5);
        sleep(3);
        falcon.collector.setPower(0);
        falcon.lift.runToPosition(-70, 1);
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
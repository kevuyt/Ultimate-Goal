package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Crater85", group = "T")
public class Crater85 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    private MasqPoint rightSample = new MasqPoint(22, -10);
    private MasqPoint leftSample = new MasqPoint(23, 9);
    private MasqPoint centerSample = new MasqPoint(24, -3, 0);
    private MasqPoint lineup = new MasqPoint(14, 39);
    private MasqPoint park = new MasqPoint(31, 21);
    public void runLinearOpMode() throws InterruptedException {
        resurrection.setStartOpenCV(true);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeAutonomous();
        resurrection.hang.setBreakMode();
        while (!opModeIsActive()) {
            resurrection.tracker.updateSystem();
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.create("H: ", resurrection.tracker.getHeading());
            dash.create("Block: ", (int) resurrection.goldAlignDetector.getXPosition());
            dash.update();
        }
        waitForStart();
        resurrection.tracker.reset();
        resurrection.unHang();
        resurrection.setLookAheadDistance(10);
        resurrection.setTimeout(5);
        BlockPlacement placement = resurrection.getBlockPlacement((int) resurrection.goldAlignDetector.getXPosition());
        resurrection.drive(2, Direction.BACKWARD);
        if (placement == BlockPlacement.CENTER) {
            resurrection.gotoXYPure(centerSample, 0, 0.9);
            resurrection.setTimeout(1);
            resurrection.gotoXYPure(14, 0, 0);
            resurrection.turnAbsolute(90, Direction.RIGHT);
            resurrection.collectionLift.lift.setPower(-1);
            sleep(1);
            resurrection.collectionLift.lift.setPower(0);
            /*resurrection.setTimeout(3);
            resurrection.setLookAheadDistance(3);
            resurrection.gotoXYPure(lineup, 30, 0.6, 0.01);
            resurrection.gotoXYPure(-16, 66, 45, 0.7, 0.05);
            resurrection.gotoXYPure(park, 45, 0.7);*/
        }
        else if (placement == BlockPlacement.LEFT) {
            resurrection.gotoXYPure(9, 0, 0, 0.7);
            resurrection.gotoXYPure(leftSample, 20, 0.8);
            resurrection.gotoXYPure(14, 0, 0);
            resurrection.turnAbsolute(90, Direction.RIGHT);
            resurrection.collectionLift.lift.setPower(-1);
            sleep(1);
            resurrection.collectionLift.lift.setPower(0);
            //resurrection.gotoXYPure(-1, 62, 45);
            //resurrection.gotoXYPure(park, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            resurrection.gotoXYPure(9, 0, 0, 0.9);
            resurrection.gotoXYPure(rightSample, 0, 0.8);
            resurrection.turnAbsolute(90, Direction.RIGHT);
            resurrection.collectionLift.lift.setPower(-1);
            sleep(1);
            resurrection.collectionLift.lift.setPower(0);
            //resurrection.gotoXYPure(12, -16, 0, 0.5);
            //resurrection.gotoXYPure(lineup, 20, 0.9, 0.007);
            //resurrection.gotoXYPure(-7, 58, 45);
            //resurrection.gotoXYPure(park, 45);
        }
    }
}

package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

@Autonomous(name = "Crater80", group = "T")
public class Crater80 extends MasqLinearOpMode  {
    private Resurrection falcon = new Resurrection();
    private MasqPoint rightSample = new MasqPoint(22, -20);
    private MasqPoint leftSample = new MasqPoint(23, 9);
    private MasqPoint centerSample = new MasqPoint(24, 0, 0);
    private MasqPoint lineup = new MasqPoint(14, 39);
    private MasqPoint park = new MasqPoint(31, 21);
    public void runLinearOpMode() throws InterruptedException {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        while (!opModeIsActive()) {
            falcon.tracker.updateSystem();
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            dash.create("H: ", falcon.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        falcon.tracker.reset();
        falcon.setLookAheadDistance(5);
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            falcon.gotoXYPure(centerSample, 0, 0.8);
            falcon.setTimeout(1);
            falcon.gotoXYPure(14, 0, 0, 0.5);
            falcon.setTimeout(3);
            falcon.gotoXYPure(lineup, 40, 1, 0.02);
            falcon.setTimeout(2);
            falcon.gotoXYPure(-16, 66, 45, 1, 0.02);
        }
        else if (placement == BlockPlacement.LEFT) {
            falcon.gotoXYPure(9, 0, 0, 0.7);
            falcon.gotoXYPure(leftSample, 20, 0.8);
            falcon.gotoXYPure(-1, 62, 45);
            falcon.gotoXYPure(park, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            falcon.gotoXYPure(9, 0, 0, 0.9);
            falcon.gotoXYPure(rightSample, 0, 0.5);
            falcon.gotoXYPure(12, -16, 0, 0.5);
            falcon.gotoXYPure(lineup, 20, 0.9, 0.007);
            falcon.gotoXYPure(-7, 58, 45);
            falcon.gotoXYPure(park, 45);
        }
    }
}
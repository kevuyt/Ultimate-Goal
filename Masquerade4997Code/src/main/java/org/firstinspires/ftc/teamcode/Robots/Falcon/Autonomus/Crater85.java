package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Crater85", group = "T")
public class Crater85 extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private MasqPoint rightSample = new MasqPoint(22, -20);
    private MasqPoint leftSample = new MasqPoint(23, 9);
    private MasqPoint centerSample = new MasqPoint(22, 0, 0);
    private MasqPoint lineup = new MasqPoint(18, 35);
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
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            falcon.gotoXY(centerSample, 0, 0.7);
            falcon.gotoXY(14, 0, 0, 0.7);
            falcon.gotoXY(lineup, 20, 0.5, 0.007);
            falcon.gotoXY(-7, 58, 45);
            falcon.gotoXY(park, 45);
        }
        else if (placement == BlockPlacement.LEFT) {
            falcon.gotoXY(9, 0, 0, 0.7);
            falcon.gotoXY(leftSample, 20, 0.8);
            falcon.gotoXY(-1, 62, 45);
            falcon.gotoXY(park, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            falcon.gotoXY(9, 0, 0, 0.9);
            falcon.gotoXY(rightSample, 0, 0.5);
            falcon.gotoXY(12, -16, 0, 0.5);
            falcon.gotoXY(lineup, 20, 0.9, 0.007);
            falcon.gotoXY(-7, 58, 45);
            falcon.gotoXY(park, 45);
        }
    }
}

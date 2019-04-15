package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Crater85", group = "T")
public class Crater85 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    private MasqPoint rightSample = new MasqPoint(22, -20);
    private MasqPoint leftSample = new MasqPoint(23, 9);
    private MasqPoint centerSample = new MasqPoint(24, 0, 0);
    private MasqPoint lineup = new MasqPoint(14, 39);
    private MasqPoint park = new MasqPoint(31, 21);
    public void runLinearOpMode() throws InterruptedException {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeAutonomous();
        while (!opModeIsActive()) {
            resurrection.tracker.updateSystem();
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            dash.create("H: ", resurrection.tracker.getHeading());
            dash.update();
        }
        waitForStart();
        resurrection.tracker.reset();
        resurrection.setLookAheadDistance(5);
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            resurrection.gotoXY(centerSample, 0);
            resurrection.setTimeout(1);
            resurrection.gotoXY(14, 0, 0, 1.5);
            resurrection.setTimeout(3);
            resurrection.setLookAheadDistance(3);
            resurrection.gotoXY(lineup, 30, 0.6, 0.01);
            resurrection.gotoXY(-16, 66, 45, 0.7, 0.05);
            resurrection.gotoXY(park, 45, 0.7);
        }
        else if (placement == BlockPlacement.LEFT) {
            resurrection.gotoXY(9, 0, 0, 0.7);
            resurrection.gotoXY(leftSample, 20, 0.8);
            resurrection.gotoXY(-1, 62, 45);
            resurrection.gotoXY(park, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            resurrection.gotoXY(9, 0, 0, 0.9);
            resurrection.gotoXY(rightSample, 0, 0.5);
            resurrection.gotoXY(12, -16, 0, 0.5);
            resurrection.gotoXY(lineup, 20, 0.9, 0.007);
            resurrection.gotoXY(-7, 58, 45);
            resurrection.gotoXY(park, 45);
        }
    }
}

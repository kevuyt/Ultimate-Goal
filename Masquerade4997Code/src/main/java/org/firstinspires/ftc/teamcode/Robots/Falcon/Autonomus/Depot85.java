package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Depot85", group = "T")
public class Depot85 extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private MasqPoint center = new MasqPoint(11, 0, -90);
    private MasqPoint rightSample = new MasqPoint(26, 238, -90);
    private MasqPoint leftSample = new MasqPoint(27, -13, -90);
    private MasqPoint marker = new MasqPoint(35, 0);
    public void runLinearOpMode() throws InterruptedException {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeAutonomous();
        falcon.lift.lift.setBreakMode();
        while (!opModeIsActive()) {
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            falcon.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        falcon.tracker.reset();
        BlockPlacement placement = BlockPlacement.RIGHT;
        if (placement == BlockPlacement.CENTER) {
            falcon.gotoXY(center);
            falcon.gotoXY(marker, -90, 0.8);
            falcon.gotoXY(5, 49, -125);
        }
        else if (placement == BlockPlacement.LEFT) {
            falcon.gotoXY(leftSample, -90,0.5);
            falcon.gotoXY(53, 4, -45);
            falcon.gotoXY(5, 49, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            falcon.gotoXY(rightSample, -90,0.5);
            falcon.gotoXY(53, 4, -135);
            falcon.gotoXY(5, 49, -135);
        }

    }
}

package org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Resources.BlockPlacement;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "DoubleSample", group = "T")
public class DoubleSample extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private MasqPoint rightSample = new MasqPoint(23, -13);
    private MasqPoint leftSample = new MasqPoint(23, 20);
    private MasqPoint centerSample = new MasqPoint(22, 0, 0);
    private MasqPoint park = new MasqPoint(25, 25);
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
            falcon.gotoXY(centerSample);
            falcon.gotoXY(14, 0, 0);
            falcon.gotoXY(17, 36, 20, 1.3);
            falcon.gotoXY(-7, 58, 45);
        }
        else if (placement == BlockPlacement.LEFT) {
            falcon.gotoXY(9, 0, 0, 0.7);
            falcon.gotoXY(leftSample, 20, 0.8);
            falcon.gotoXY(-1, 62, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            falcon.gotoXY(9, 0, 0, 0.7);
            falcon.gotoXY(rightSample, 0, 0.1);
            falcon.gotoXY(13, -12, 0);
            falcon.gotoXY(17, 36, 20, 1.3, 0.007);
            falcon.gotoXY(-7, 58, 45);
        }

    }
}
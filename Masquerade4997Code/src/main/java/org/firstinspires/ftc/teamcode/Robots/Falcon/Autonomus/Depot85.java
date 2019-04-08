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
    private MasqPoint centerSample = new MasqPoint(22, 0, 90);
    private MasqPoint center = new MasqPoint(11, 0, -90);
    private MasqPoint rightSample = new MasqPoint(19, 14, -90);
    private MasqPoint leftSample = new MasqPoint(-14, 23, -90);
    private MasqPoint marker = new MasqPoint(35, 0);
    public void runLinearOpMode() throws InterruptedException {
        falcon.setStartOpenCV(false);
        falcon.mapHardware(hardwareMap);
        falcon.initializeTeleop();
        falcon.lift.lift.setBreakMode();
        while (!opModeIsActive()) {
            dash.create("X: ", falcon.tracker.getGlobalX());
            dash.create("Y: ", falcon.tracker.getGlobalY());
            falcon.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        falcon.lift.lift.setPower(1);
        sleep(1);
        falcon.lift.lift.setPower(0);
        falcon.tracker.reset();
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            falcon.gotoXY(center, -90);
            falcon.gotoXY(marker, -90);
            falcon.gotoXY(center, -90);
        }
        else if (placement == BlockPlacement.LEFT) {
        }
        else if (placement == BlockPlacement.RIGHT) {

        }

    }
}

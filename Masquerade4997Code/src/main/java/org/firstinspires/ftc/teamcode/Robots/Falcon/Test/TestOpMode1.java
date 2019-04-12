package org.firstinspires.ftc.teamcode.Robots.Falcon.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Autonomus.Constants;
import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "TestOpMode1", group = "T")
public class TestOpMode1 extends MasqLinearOpMode implements Constants {
    private Falcon falcon = new Falcon();
    private MasqPoint lineup = new MasqPoint(0, 30, 0);
    private MasqPoint marker = new MasqPoint(-30, 30, 90);
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
        falcon.setTimeout(5);
        falcon.gotoXY(0, -50, -175, 1, 0.1);

    }
}

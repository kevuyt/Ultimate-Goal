package org.firstinspires.ftc.teamcode.Robots.Reserection.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "TestOpMode1", group = "T")
@Disabled
public class TestOpMode1 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    private MasqPoint lineup = new MasqPoint(0, 30, 0);
    private MasqPoint marker = new MasqPoint(-30, 30, 90);
    public void runLinearOpMode() throws InterruptedException {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeTeleop();
        resurrection.lift.lift.setBreakMode();
        while (!opModeIsActive()) {
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            resurrection.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        resurrection.setTimeout(5);
        resurrection.path(0.7, lineup, marker);

    }
}

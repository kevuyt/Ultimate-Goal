package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;

import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Depot85", group = "T")
public class Depot85 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    private MasqPoint center = new MasqPoint(11, 0, -90);
    private MasqPoint rightSample = new MasqPoint(26, 238, -90);
    private MasqPoint leftSample = new MasqPoint(27, -13, -90);
    private MasqPoint marker = new MasqPoint(35, 0);
    public void runLinearOpMode() throws InterruptedException {
        resurrection.setStartOpenCV(false);
        resurrection.mapHardware(hardwareMap);
        resurrection.initializeAutonomous();
        while (!opModeIsActive()) {
            dash.create("X: ", resurrection.tracker.getGlobalX());
            dash.create("Y: ", resurrection.tracker.getGlobalY());
            resurrection.tracker.updateSystem();
            dash.update();
        }
        waitForStart();
        resurrection.tracker.reset();
        BlockPlacement placement = BlockPlacement.CENTER;
        if (placement == BlockPlacement.CENTER) {
            resurrection.gotoXY(center);
            resurrection.gotoXY(marker, -90, 0.8);
            resurrection.gotoXY(5, 49, -125);
        }
        else if (placement == BlockPlacement.LEFT) {
            resurrection.gotoXY(leftSample, -90,0.5);
            resurrection.gotoXY(53, 4, -45);
            resurrection.gotoXY(5, 49, 45);
        }
        else if (placement == BlockPlacement.RIGHT) {
            resurrection.gotoXY(rightSample, -90,0.5);
            resurrection.gotoXY(53, 4, -135);
            resurrection.gotoXY(5, 49, -135);
        }

    }
}

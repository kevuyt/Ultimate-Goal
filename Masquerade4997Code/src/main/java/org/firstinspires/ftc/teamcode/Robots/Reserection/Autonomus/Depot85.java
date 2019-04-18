package org.firstinspires.ftc.teamcode.Robots.Reserection.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Reserection.Resources.BlockPlacement;
import org.firstinspires.ftc.teamcode.Robots.Reserection.Resurrection;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqResources.MasqMath.MasqPoint;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "Depot85", group = "T")
public class Depot85 extends MasqLinearOpMode {
    private Resurrection resurrection = new Resurrection();
    private MasqPoint center = new MasqPoint(11, 0, -90);
    private MasqPoint rightSample = new MasqPoint(26, 18, -90);
    private MasqPoint leftSample = new MasqPoint(27, -13, -90);
    private MasqPoint marker = new MasqPoint(44, 3);
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
        BlockPlacement placement = resurrection.getBlockPlacement((int) resurrection.goldAlignDetector.getXPosition());
        resurrection.unHang();
        resurrection.drive(3, Direction.BACKWARD);
        resurrection.setLookAheadDistance(5);
        if (placement == BlockPlacement.CENTER) {
            resurrection.gotoXY(marker, -90, 0.8);
            resurrection.gotoXY(center, -90, 0.6);
            resurrection.gotoXYPure(12, 42, 0, 0.6);
        }
        else if (placement == BlockPlacement.LEFT) {
            resurrection.gotoXYPure(leftSample, -90,0.5);
            resurrection.gotoXYPure(53, 4, -45, 0.5);
            resurrection.turnRelative(90, Direction.RIGHT);
            resurrection.gotoXYPure(12, 42, -135, 0.5);
        }
        else if (placement == BlockPlacement.RIGHT) {
            resurrection.gotoXYPure(rightSample, -90,0.5);
            resurrection.gotoXYPure(53, 4, -135, 0.5);
            resurrection.gotoXYPure(12, 42, -135, 0.5);
        }

    }
}

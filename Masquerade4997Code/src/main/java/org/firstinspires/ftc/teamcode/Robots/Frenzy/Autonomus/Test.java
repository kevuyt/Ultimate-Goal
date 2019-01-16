package org.firstinspires.ftc.teamcode.Robots.Frenzy.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robots.Frenzy.Frenzy;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 1/5/19.
 * Project: MasqLib
 */
@Autonomous(name = "Test", group = "Tank")
@Disabled
public class Test extends MasqLinearOpMode {
    Frenzy frenzy = new Frenzy();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        frenzy.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("G");
            dash.update();
        }
        waitForStart();
        frenzy.turnAbsolute(90, Direction.LEFT);
        frenzy.turnAbsolute(0, Direction.RIGHT);
    }
}

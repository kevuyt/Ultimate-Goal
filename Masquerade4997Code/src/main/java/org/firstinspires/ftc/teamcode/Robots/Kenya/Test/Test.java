package org.firstinspires.ftc.teamcode.Robots.Kenya.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Kenya.Kenya;

import Library4997.MasqResources.MasqHelpers.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 3/21/19.
 * Project: MasqLib
 */
@Autonomous(name = "Test", group = "Autonomus")
public class Test extends MasqLinearOpMode {
    Kenya robot = new Kenya();
    public void runLinearOpMode() throws InterruptedException {

        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("hello");
            dash.update();
        }
        waitForStart();
        int iterations = 0;
        while (iterations < 4) {
            robot.drive(3 * 12);
            robot.turnRelative(90, Direction.RIGHT);
            iterations = iterations + 1;
        }

    }
}
package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "MasqGeneralTester", group = "Autonomus")
public class MasqGeneralTester extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Jewel Color Red: ", robot.jewelColorRed.stop());
            dash.update();
        }
        waitForStart();
        robot.go(5, 90, Direction.RIGHT, 0, Direction.BACKWARD);
        robot.go(10, 90, Direction.RIGHT, 0, Direction.BACKWARD);

    }
}
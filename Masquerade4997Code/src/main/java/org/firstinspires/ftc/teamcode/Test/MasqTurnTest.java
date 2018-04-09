package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "MasqTurnTest", group = "Autonomus")
public class MasqTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);

        robot.sleep(100);
        while (!opModeIsActive()) {
            dash.create("Hola ");
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.drive(48, POWER_OPTIMAL);
        robot.turnRelative(90, Direction.LEFT);
        robot.turnRelative(90, Direction.RIGHT);
        robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
    }
}
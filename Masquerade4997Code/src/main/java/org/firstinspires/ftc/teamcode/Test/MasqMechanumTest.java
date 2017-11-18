package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/18/17.
 */

@Autonomous(name = "MasqMechanumTest", group = "Autonomus")
public class MasqMechanumTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            robot.MECH(controller1);
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        int counter  = 0;
        while (counter < 16) {
            robot.turn(30, Direction.LEFT);
            counter++;
        }
        while (counter < 8) {
            robot.turn(45, Direction.LEFT);
            counter++;
        }
        while (counter < 4) {
            robot.turn(90, Direction.LEFT);
            counter++;
        }
    }
}

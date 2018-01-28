package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "MasqTurnTest", group = "Autonomus")
public class MasqTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        double columnCount = 0, firstOptoReading = robot.ods.rawLight(), OPTOSENSOR_DETECT_CRYPTOBOX_THRESHOLD = 1.395, TIME_BETWEEN_OPTOSENSOR_DETECTIONS = 1500, stopState = 0;
        long startTime = System.nanoTime();
        robot.blueRotator.setPosition(ROTATOR_BLUE_OUT);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        while (!opModeIsActive()) {
            dash.create(robot.jewelColorBlue);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.stop(robot.jewelColorBlue, .1, Direction.BACKWARD);
        /*
        robot.drive(48, POWER_OPTIMAL);
        robot.turn(90, Direction.LEFT);
        robot.turn(90, Direction.RIGHT);
        robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);*/
    }
}
package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/8/17.
 */
@Autonomous(name = "MasqServoTest", group = "Autonomus")
@Disabled
public class MasqServoTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1000);
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
    }
}
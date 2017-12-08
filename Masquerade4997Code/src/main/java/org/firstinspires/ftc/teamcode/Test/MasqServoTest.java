package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/8/17.
 */
@Autonomous(name = "MasqServoTest", group = "Autonomus")
public class MasqServoTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.blueRotator.setPosition(COLOR_SEEN);
        robot.redRotator.setPosition(COLOR_SEEN);
        robot.sleep(1000);
        robot.blueRotator.setPosition(COLOR_NOT_SEEN);
        robot.redRotator.setPosition(COLOR_NOT_SEEN);
        robot.sleep(1000);
        robot.blueRotator.setPosition(ROTATOR_CENTER);
        robot.redRotator.setPosition(ROTATOR_CENTER);
    }
}
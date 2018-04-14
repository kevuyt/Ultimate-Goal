package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "MasqTurnTest", group = "Autonomus")
public class MasqTurnTest extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.yWheel.resetEncoder();
        robot.setYTarget((int) robot.yWheel.getPosition());
        while (!opModeIsActive()) {
            dash.create("Position: ", robot.yWheel.getPosition());
            dash.create("Velocity: ", robot.intake.motor1.getVelocity());
            dash.update();
        }
        waitForStart();
    }
}
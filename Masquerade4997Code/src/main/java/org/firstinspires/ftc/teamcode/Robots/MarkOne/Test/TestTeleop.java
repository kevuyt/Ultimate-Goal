package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/17/2019
 */
@TeleOp(name = "TestTeleop", group = "MarkOne")
public class TestTeleop extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        robot.foundationHook.raise();
        while(opModeIsActive()) {
            robot.MECH(controller1);
            dash.create(robot.tracker);
            dash.update();
            controller1.update();
        }
    }
}

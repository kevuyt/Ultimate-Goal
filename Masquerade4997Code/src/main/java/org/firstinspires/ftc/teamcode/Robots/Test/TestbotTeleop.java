package org.firstinspires.ftc.teamcode.Robots.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/28/2019
 */
@TeleOp(name = "TestBotTeleop", group = "Test")
public class TestbotTeleop extends MasqLinearOpMode {
    TestRobot robot = new TestRobot();
    @Override
    public void runLinearOpMode() {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create("Big Brain Time");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.MECH(controller1);
        }
    }
}

package org.firstinspires.ftc.teamcode.Robots.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/28/2019
 */
@TeleOp(name = "TestBotTeleop", group = "Test")
public class TestbotTeleop extends MasqLinearOpMode {
    @Override
    public void runLinearOpMode() {
        TestRobot robot = new TestRobot();
        while (!opModeIsActive()) {
            robot.mapHardware(hardwareMap);
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.MECH(controller1);
        }
    }
}

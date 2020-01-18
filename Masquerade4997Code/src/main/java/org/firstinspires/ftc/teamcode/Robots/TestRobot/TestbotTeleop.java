package org.firstinspires.ftc.teamcode.Robots.TestRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 9/28/2019
 */
@TeleOp(name = "TestBotTeleop", group = "Testbot")
@Disabled
public class TestbotTeleop extends MasqLinearOpMode {
    private TestRobot robot = new TestRobot();
    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap);
        robot.initializeTeleop();
        while (!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.MECH(controller1);
            /*if (controller1.leftTriggerPressed()) robot.intake.setVelocity(-1);
            else if (controller1.rightTriggerPressed()) robot.intake.setVelocity(-1);
            else robot.intake.setVelocity(0);*/

            controller1.update();
        }
    }
}

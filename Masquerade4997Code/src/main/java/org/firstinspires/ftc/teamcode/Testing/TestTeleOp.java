package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends MasqLinearOpMode {
    private TestBot robot = new TestBot();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("VS: ", robot.shooter.getVelocity());
            robot.shooter.setVelocity(controller1.rightTrigger()-controller1.leftTrigger());

            dash.create("VI: ", robot.intake.getVelocity());
            if(controller1.rightBumper()) robot.intake.setVelocity(1);
            else if (controller1.leftBumper()) robot.intake.setVelocity(0);

            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(controller1);
        }
    }
}
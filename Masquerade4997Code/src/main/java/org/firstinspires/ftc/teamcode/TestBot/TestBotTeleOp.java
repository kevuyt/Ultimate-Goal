package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/9/2020
 */
@TeleOp(name = "TestBotTeleOp", group = "Osiris")
public class TestBotTeleOp extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(!opModeIsActive()) {
            dash.create("Initialized");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH();
            robot.intake.setVelocity(controller1.rightTrigger()-controller1.leftTrigger());
            if(controller1.y()) robot.shooter.setVelocity(-1);
            else robot.shooter.setVelocity(0);
        }
    }
}

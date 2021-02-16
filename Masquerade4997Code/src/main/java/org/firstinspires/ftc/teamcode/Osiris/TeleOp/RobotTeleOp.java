package org.firstinspires.ftc.teamcode.Osiris.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqWrappers.MasqLinearOpMode;

import static java.lang.Math.abs;

/**
 * Created by Keval Kataria on 11/9/2020
 */
@TeleOp(name = "RobotTeleOp", group = "Osiris")
public class RobotTeleOp extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private boolean fieldCentric = false;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(!opModeIsActive()) {
            dash.create("Initialized");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH(fieldCentric);

            robot.intake.setVelocity(controller1.rightTrigger()-controller1.leftTrigger());

            if(controller1.y()) {
                robot.shooter.setVelocity(-0.7);
                robot.hopper.setPosition(1);

            }
            else {
                robot.shooter.setVelocity(0);
                robot.hopper.setPosition(0);
            }

            if(controller1.dPadUp()) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);

            if(controller1.dPadRight()) fieldCentric = true;
            else if (controller1.dPadLeft()) fieldCentric = false;


            dash.update();
        }
    }
}
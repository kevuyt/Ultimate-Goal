package org.firstinspires.ftc.teamcode.Osiris.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 11/9/2020
 */

@TeleOp(group = "Main")
public class RobotTeleOp extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    double shooterSpeed = 0.66, intakeSpeed = 0.5;
    String mode = "GOAL";
    boolean enabled = false;

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        dash.create("Initialized");
        dash.update();

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH();

            if(!enabled) robot.intake.setPower(intakeSpeed * gamepad1.left_trigger - gamepad1.right_trigger);

            if(gamepad1.left_bumper) {
                robot.shooter.setPower(shooterSpeed);
                robot.hopper.setPosition(1);
                enabled = true;
            }
            else {
                robot.shooter.setPower(0);
                robot.hopper.setPosition(0);
                enabled = false;
            }

            if(gamepad1.right_bumper && enabled) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);

            if(gamepad1.dpad_left)  shooterSpeed -= 0.0001;
            else if(gamepad1.dpad_right) shooterSpeed += 0.0001;
            if(gamepad1.dpad_down)  intakeSpeed -= 0.0001;
            else if(gamepad1.dpad_up) intakeSpeed += 0.0001;

            robot.claw.driverControl(gamepad1);

            dash.create("Shooter Speed:", shooterSpeed);
            dash.create("Intake Speed:", intakeSpeed);
            //dash.create("Shooter Mode:", mode);
            //dash.create("Rings in Hopper:", robot.getRings());
            dash.update();
        }
    }
}
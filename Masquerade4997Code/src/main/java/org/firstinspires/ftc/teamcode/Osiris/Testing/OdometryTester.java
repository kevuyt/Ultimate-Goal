package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "OdometryTester", group = "Test")
public class OdometryTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    double shooterSpeed = 0.6;
    String mode;
    boolean enabled = false;

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, TELEOP);
        robot.driveTrain.setClosedLoop(false);

        while (!opModeIsActive()) {
            robot.tracker.updateSystem();

            dash.create(robot.tracker);
            dash.create("Raw X: ", robot.intake.getInches());
            dash.create("Raw YL: ", robot.shooter.getInches());
            dash.create("Raw YR: ", robot.encoder.getInches());
            dash.update();
        }

        waitForStart();

        robot.driveTrain.setClosedLoop(true);

        while(opModeIsActive()) {
            robot.MECH();

            if(!enabled) robot.intake.setVelocity(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.left_bumper) {
                robot.shooter.setVelocity(shooterSpeed);
                robot.hopper.setPosition(1);
                enabled = true;
            }
            else {
                robot.shooter.setVelocity(0);
                robot.hopper.setPosition(0);
                enabled = false;
            }

            if(gamepad1.right_bumper && enabled) robot.flicker.setPosition(0.85);
            else robot.flicker.setPosition(0);

            if(gamepad1.dpad_left) {
                shooterSpeed = 0.58;
                mode = "POWER SHOT";
            }
            else if(gamepad1.dpad_right) {
                shooterSpeed = 0.66;
                mode = "GOAL";
            }
            else if(gamepad1.dpad_up) {
                shooterSpeed = 0.70;
                mode = "HIGH POWER";
            }

            robot.claw.driverControl(gamepad1);

            robot.tracker.updateSystem();

            dash.create(robot.tracker);
            dash.create("Shooter Mode: " + mode);
            dash.update();
        }
    }
}
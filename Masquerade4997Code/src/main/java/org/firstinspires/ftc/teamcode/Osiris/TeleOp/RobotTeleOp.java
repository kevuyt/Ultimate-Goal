package org.firstinspires.ftc.teamcode.Osiris.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 11/9/2020
 */

@TeleOp(group = "Main")
public class RobotTeleOp extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    private String mode = "GOAL";
    private double shooterPower = 0.5;
    private boolean enabled = false;

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        dash.create("Initialized");
        dash.update();

        waitForStart();

        while(opModeIsActive()) {
            robot.MECH();

            if(!enabled && gamepad1.left_trigger > 0) robot.intake.setPower(1);
            else if(!enabled && gamepad1.right_trigger > 0) robot.intake.setPower(-1);
            else robot.intake.setPower(0);

            if(gamepad1.left_bumper) {
                robot.shooter.setPower(shooterPower);
                robot.hopper.setPosition(1);
                robot.claw.close();
                Thread thread = new Thread(() -> {
                    sleep(1000);
                    robot.compressor.setPosition(1);
                });
                thread.start();
                enabled = true;
            }
            else {
                robot.shooter.setPower(0);
                robot.hopper.setPosition(0);
                robot.compressor.setPosition(0);
                enabled = false;
            }

            if(gamepad1.right_bumper && enabled) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);

            if(gamepad1.dpad_left) {
                mode = "POWER_SHOT";
                shooterPower = 0.426;
            }
            else if(gamepad1.dpad_right) {
                mode = "GOAL";
                shooterPower = 0.5;
            }
            else if(gamepad1.dpad_up) {
                mode = "HIGH_POWER";
                shooterPower = 0.55;
            }

            robot.claw.driverControl(gamepad1);

            dash.create("Shooter Mode:", mode);
            dash.create("Rings in Hopper:", robot.getRings());
            dash.update();
        }
    }
}
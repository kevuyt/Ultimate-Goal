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
    private final Osiris robot = new Osiris();
    private String mode = "GOAL";
    private double shooterPower = 0.7;
    private Thread thread = new Thread(), thread2 = new Thread();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        dash.create("Initialized");
        dash.update();

        waitForStart();

        robot.tracker.imu.reset();

        while(opModeIsActive()) {
            robot.easyTurnMech(true);

            if(gamepad1.left_bumper) {
                robot.shooter.setPower(shooterPower);
                robot.hopper.setPosition(1);
                robot.claw.close();
                if (!thread.isAlive()) {
                    thread = new Thread(() -> {
                        sleep();
                        robot.compressor.setPosition(1);
                    });
                    thread.start();
                }
                if(gamepad1.y) {
                    if (!thread2.isAlive()) {
                        thread2 = new Thread(this::shoot);
                        thread2.start();
                    }
                }
                else if(gamepad1.right_bumper) robot.flicker.setPosition(1);
                else if(!thread2.isAlive()) robot.flicker.setPosition(0);
                robot.intake.setPower(0);
                robot.aligner.setPosition(0);
            }
            else {
                robot.shooter.setPower(0);
                robot.hopper.setPosition(0);
                if(thread.isAlive()) thread.interrupt();
                if(thread2.isAlive()) thread2.interrupt();
                robot.compressor.setPosition(0);
                robot.claw.driverControl(gamepad1);
                robot.flicker.setPosition(0);
                if(gamepad1.left_trigger > 0) robot.intake.setPower(1);
                else if(gamepad1.right_trigger > 0) robot.intake.setPower(-1);
                else robot.intake.setPower(0);
                robot.aligner.setPosition(gamepad1.x ? 1 : 0);
            }

            if(gamepad1.dpad_left) {
                mode = "POWER_SHOT";
                shooterPower = 0.58;
            }
            else if(gamepad1.dpad_up) {
                mode = "HIGH_POWER";
                shooterPower = 0.75;
            }
            else if(gamepad1.dpad_right) {
                mode = "GOAL";
                shooterPower = 0.7;
            }

            /*
            if(gamepad1.dpad_left) shooterPower -= 0.005;
            if(gamepad1.dpad_right) shooterPower += 0.005;
            if(gamepad1.right_stick_button) intakePower += 0.005;
            if(gamepad1.left_stick_button) intakePower -= 0.005;
            dash.create(robot.shooter, robot.intake);
            */

            dash.create("Shooter Mode:", mode);
            dash.create(robot.tracker.imu);
            dash.update();
        }
    }
    public void shoot() {
        for(int i = 0; i < 3; i++) {
            robot.flicker.setPosition(1);
            sleep();
            robot.flicker.setPosition(0);
            sleep();
        }
    }
}
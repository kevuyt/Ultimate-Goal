package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqWrappers.MasqLinearOpMode;

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
    public void runLinearOpMode() throws InterruptedException {
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
            robot.tracker.updateSystem();

            robot.MECH();

            robot.intake.setVelocity(controller1.rightTrigger()-controller1.leftTrigger());

            if(controller1.leftBumper()) {
                robot.shooter.setVelocity(shooterSpeed);
                robot.hopper.setPosition(1);
                enabled = true;
            }
            else {
                robot.shooter.setVelocity(0);
                robot.hopper.setPosition(0);
                enabled = false;
            }

            if(controller1.rightBumper() && enabled) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);

            if(controller1.dPadLeft()) shooterSpeed = 0.54;
            if(controller1.dPadRight()) shooterSpeed = 0.6;

            robot.claw.driverControl(controller1);

            mode = shooterSpeed == 0.54 ? "POWERSHOT" : "GOAL";

            dash.create("Shooter Mode: " + mode);
            dash.create(robot.tracker);
            dash.update();
        }
    }
}
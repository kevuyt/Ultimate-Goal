package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "OdometryTester", group = "Test")
public class OdometryTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();
    double value = 0.58;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
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

            if(controller1.y()) {
                robot.shooter.setVelocity(value);
                robot.hopper.setPosition(1);

            }
            else {
                robot.shooter.setVelocity(0);
                robot.hopper.setPosition(0);
            }

            if(controller1.dPadUp()) robot.flicker.setPosition(1);
            else robot.flicker.setPosition(0);
            if(controller1.dPadLeft()) value -= 0.0001;
            if(controller1.dPadRight()) value += 0.0001;

            dash.create(robot.tracker);
            dash.create("Shooter: ", value);
            dash.update();
        }
    }
}
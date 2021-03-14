package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "OdometryTester", group = "Test")
@Disabled
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
            dash.create("Raw YL: ", robot.encoder1.getInches());
            dash.create("Raw YR: ", robot.encoder2.getInches());
            dash.update();
        }

        waitForStart();

        robot.driveTrain.setClosedLoop(true);

        while(opModeIsActive()) {
            robot.MECH();

            robot.tracker.updateSystem();

            dash.create(robot.tracker);
            dash.create("Shooter Mode: " + mode);
            dash.update();
        }
    }
}
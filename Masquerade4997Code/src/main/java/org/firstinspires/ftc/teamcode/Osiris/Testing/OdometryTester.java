package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "OdometryTester", group = "Test")
@Disabled
public class OdometryTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, TELEOP);
        robot.driveTrain.setVelocityControl(false);

        while (!opModeIsActive()) {
            robot.tracker.updateSystem();

            dash.create(robot.tracker);
            dash.create("Raw X: ", robot.intake.getInches());
            dash.create("Raw YL: ", robot.encoder1.getInches());
            dash.create("Raw YR: ", robot.encoder2.getInches());
            dash.update();
        }

        waitForStart();

        robot.driveTrain.setVelocityControl(true);

        while(opModeIsActive()) {
            robot.MECH();

            robot.tracker.updateSystem();

            dash.create(robot.tracker);
            dash.update();
        }
    }
}
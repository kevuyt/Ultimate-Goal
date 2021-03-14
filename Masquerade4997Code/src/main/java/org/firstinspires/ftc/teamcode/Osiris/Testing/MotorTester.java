package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 3/14/2021
 */
@TeleOp(name = "MotorTester", group = "Test")
@Disabled
public class MotorTester extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, TELEOP);

        while(!opModeIsActive()) {
            robot.MECH();

            dash.create("DriveTrain is in custom PID mode");
            dash.update();
        }

        waitForStart();

        robot.driveTrain.setClosedLoop(false);
        robot.driveTrain.runUsingEncoder();

        while(opModeIsActive()) {
            robot.MECH();
            dash.create("DriveTrain is in build in PID mode");
            dash.update();
        }
    }
}

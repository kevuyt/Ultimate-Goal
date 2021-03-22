package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqueradeLibrary.MasqMotion.MasqServoProgrammer;
import MasqueradeLibrary.MasqResources.MasqLinearOpMode;

import static MasqueradeLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */
@TeleOp(name = "ServoProgrammer", group = "Test")
public class ServoProgrammer extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);
        MasqServoProgrammer servoProgrammer = new MasqServoProgrammer(robot.claw.getClaw(), robot.claw.getRotater(), robot.hopper, robot.flicker);

        dash.create("Hello, this is the Servo Programmer");
        dash.update();

        waitForStart();

        while(opModeIsActive()) servoProgrammer.run();
    }
}
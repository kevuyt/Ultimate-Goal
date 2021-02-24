package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import Library4997.MasqServos.MasqServoProgrammer;
import Library4997.MasqResources.MasqLinearOpMode;

import static Library4997.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */
@TeleOp(name = "ServoProgrammer", group = "Test")
public class ServoProgrammer extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, TELEOP);
        MasqServoProgrammer servoProgrammer = new MasqServoProgrammer(robot.claw.getClaw(), robot.claw.getRotater());

        dash.create("Hello, this is the Servo Programmer");
        dash.update();

        waitForStart();

        while(opModeIsActive()) servoProgrammer.run();
    }
}
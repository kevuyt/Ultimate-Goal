package org.firstinspires.ftc.teamcode.PlaceHolder.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqServos.MasqServoProgrammer;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 12/30/2020
 */
@TeleOp(name = "ClawProgrammer", group = "Test")
public class ClawProgrammer extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MasqServoProgrammer servoProgrammer = new MasqServoProgrammer(robot.claw.claw, robot.claw.rotater);

        dash.create("Hello, this is the Claw Programmer");
        dash.update();

        waitForStart();

        while(opModeIsActive()) {
            servoProgrammer.run();
        }
    }
}
package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqServos.MasqServoProgrammer;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 12/30/2020
 */
@TeleOp(name = "ClawProgrammer", group = "Test")
public class ClawProgrammer extends MasqLinearOpMode {
    private TestBot robot = new TestBot();
    private MasqServoProgrammer servoProgrammer;

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        servoProgrammer = new MasqServoProgrammer(robot.claw.claw, robot.claw.rotater)

        while (!opModeIsActive()) {
            dash.create("Hello, this is the Claw Programmer");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            servoProgrammer.run();
        }
    }
}

package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqMotion.MasqServoProgrammer;
import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 12/30/2020
 */

@TeleOp
public class ServoProgrammer extends MasqLinearOpMode {
    private Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);
        MasqServoProgrammer servoProgrammer = new MasqServoProgrammer(robot.claw.getClaw(), robot.claw.getRotator(), robot.hopper, robot.flicker);

        while(!opModeIsActive()) {
            servoProgrammer.init();

            if(isStopRequested()) break;
        }

        waitForStart();

        while(opModeIsActive()) servoProgrammer.run();
    }
}
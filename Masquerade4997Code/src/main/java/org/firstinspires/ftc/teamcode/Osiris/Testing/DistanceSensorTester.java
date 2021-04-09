package org.firstinspires.ftc.teamcode.Osiris.Testing;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.Osiris.Robot.Osiris;

import MasqLibrary.MasqResources.MasqLinearOpMode;

import static MasqLibrary.MasqRobot.OpMode.TELEOP;

/**
 * Created by Keval Kataria on 3/22/2021
 */

@TeleOp(group = "Test")
public class DistanceSensorTester extends MasqLinearOpMode {
    private final Osiris robot = new Osiris();

    @Override
    public void runLinearOpMode() {
        robot.init(TELEOP);

        while (!isStopRequested()) {
            dash.create(robot.distanceSensor);
            dash.update();
        }
    }
}
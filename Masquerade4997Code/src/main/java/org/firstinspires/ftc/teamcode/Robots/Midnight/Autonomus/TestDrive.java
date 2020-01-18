package org.firstinspires.ftc.teamcode.Robots.Midnight.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@Autonomous(name = "TestDrive", group = "Testbot")
public class TestDrive extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeAutonomous();

        while(!opModeIsActive()) {
            dash.create("Left Position: ", robot.driveTrain.leftDrive.getCurrentPosition());
            dash.create("Right Position: ", robot.driveTrain.rightDrive.getCurrentPosition());
            dash.update();
        }

        waitForStart();
        robot.drive(10);
    }
}

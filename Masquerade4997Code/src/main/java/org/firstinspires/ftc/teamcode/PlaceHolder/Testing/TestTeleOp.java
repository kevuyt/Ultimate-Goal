package org.firstinspires.ftc.teamcode.PlaceHolder.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PlaceHolder.Robot.PlaceHolder;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Keval Kataria on 11/22/2020
 */

@TeleOp(name = "TestTeleOp", group = "Test")
public class TestTeleOp extends MasqLinearOpMode {
    private PlaceHolder robot = new PlaceHolder();

    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("VS: ", robot.shooter.getPower());
            if(controller1.a()) robot.shooter.setVelocity(1);
            else robot.shooter.setVelocity(0);

            dash.create("VI: ", robot.intake.getPower());
            if(controller1.rightBumper()) robot.intake.setVelocity(1);
            else if (controller1.leftBumper()) robot.intake.setVelocity(0);

            dash.update();
        }

        waitForStart();

        robot.driveTrain.setClosedLoop(false);

        while(opModeIsActive()) {
            robot.tracker.updateSystem();
            dash.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.Robots.Midnight.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Midnight.Robot.Midnight;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archishmaan Peyyety on 2020-01-17.
 * Project: MasqLib
 */
@TeleOp(name = "MidnightTest", group = "ZZZ")
@Disabled

public class MidnightTest extends MasqLinearOpMode {
    private Midnight robot = new Midnight();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.initializeTeleop();

        while(!opModeIsActive()) {
            dash.create("Heading: ", robot.tracker.getHeading());
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (controller1.y()) robot.driveTrain.setPower(1, 1);
            else if (controller1.a()) robot.driveTrain.setPower(-1,-1);
            else if (controller1.x()) robot.driveTrain.setPower(-1, 0);
            else if (controller1.b()) robot.driveTrain.setPower(1, 0);
            else robot.driveTrain.setPower(0, 0);
        }
    }
}

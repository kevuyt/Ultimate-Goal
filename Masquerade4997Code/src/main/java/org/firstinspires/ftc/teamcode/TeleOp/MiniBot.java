package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqMotors.MasqTankDrive;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 5/13/17.
 */
@TeleOp(name="MiniBot", group="Final")
@Disabled
public class MiniBot extends LinearOpMode {
    private MasqTankDrive robot = new MasqTankDrive("Left Front", "Left Back", "Right Front", "Right Back");

    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted()) {
            telemetry.addLine("COUNT");
            telemetry.update();
        }
        double power = 0;
        waitForStart();
        while (opModeIsActive()) {
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double left = move + turn;
            double right = move - turn;

            if (left > 1.0) {
                left /= left;
                right /= left;
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            } else if (right > 1.0) {
                left /= right;
                right /= right;
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            } else {
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            }
        }
    }
}

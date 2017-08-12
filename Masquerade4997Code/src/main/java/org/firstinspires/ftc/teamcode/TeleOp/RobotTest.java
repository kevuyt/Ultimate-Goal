package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqMotors.MasqTankDrive;

/**
 * Created by Archish on 7/15/17.
 */
@TeleOp(name="R2", group="Final")
public class RobotTest extends LinearOpMode {
    public MasqTankDrive robot = new MasqTankDrive("leftFront", "leftBack", "rightBack", "rightFront");
    @Override
    public void runOpMode() throws InterruptedException {
        while (!isStarted()) {
            telemetry.addLine("Hello Faggot");
            telemetry.update();
        }
        while (opModeIsActive()) {
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double left = move - turn;
            double right = move + turn;
            if(left > 1.0) {
                left /= left;
                right /= left;
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            }
            else if (right > 1.0) {
                left /= right;
                right /= right;
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            }
            else {
                robot.setPowerLeft(-left);
                robot.setPowerRight(-right);
            }
        }
    }
}

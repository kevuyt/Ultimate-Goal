package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.RobotTwo;


/**
 * This is a basic template copy and paste this class for any TeleOp,
 * refactor the file name to match the TeleOp class title
 */

@TeleOp(name = "DefenseTeleOp", group = "Template")
@Disabled
public class DefenseTeleOp extends LinearOpMode implements Constants{
    private RobotTwo robot = new RobotTwo();
    public void runOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            telemetry.addLine("Hello");
            telemetry.update();
        }
        waitForStart();
        while (opModeIsActive()){
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double left = move - turn;
            double right = move + turn;
            if(left > 1.0) {
                left /= left;
                right /= left;
                robot.driveTrain.setPowerLeft(-left);
                robot.driveTrain.setPowerRight(-right);
            }
            else if (right > 1.0) {
                left /= right;
                right /= right;
                robot.driveTrain.setPowerLeft(-left);
                robot.driveTrain.setPowerRight(-right);
            }
            else {
                robot.driveTrain.setPowerLeft(-left);
                robot.driveTrain.setPowerRight(-right);
            }
        }
    }
}

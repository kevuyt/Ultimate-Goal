package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import BasicLib4997.MasqLinearOpMode;
import BasicLib4997.MasqMotors.MasqRobot.Direction;

/**
 * A Template to follow for all TeleOp Opmodes
 */

@TeleOp(name = "Example-Teleop", group = "Test")
public class ExampleTeleOp extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            telemetry.addLine("Status");
            telemetry.update();
        }
        double shooterPower = 0;
        boolean sweepPressed = false;
        double sweepPower = 0;
        waitForStart();
        while (opModeIsActive()){
            robot.leftPresser.setPosition(1);
            robot.rightPresser.setPosition(.03);
            float right = -controller1.left_stick_y();
            float left = -controller1.right_stick_y();

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);


            float speedCheck = controller1.left_trigger();

            speedCheck = Range.clip(speedCheck, 0, 1);

            if (speedCheck > 0.5) {
                robot.driveTrain.setPowerRight(right * SCALEDOWN);
                robot.driveTrain.setPowerLeft(left * SCALEDOWN);
            } else {
                robot.driveTrain.setPowerRight(-1 * right);
                robot.driveTrain.setPowerLeft(-1 * left);
            }
            if (controller2.b()) {
                robot.indexer.setPosition(.3);

                waitNow(400);
                robot.indexer.setPosition(0);

            }
            if (controller2.dpad_down() && !sweepPressed) {
                sweepPower = 0.9;
                sweepPressed = true;
            } else if (controller2.dpad_up() && !sweepPressed) {
                sweepPower = -0.9;
                sweepPressed = true;
            } else if (controller2.dpad_left() && sweepPressed) {
                sweepPower = 0;
                sweepPressed = false;
            }
            robot.collector.setPower(sweepPower);

            if (controller1.dpad_down() && !sweepPressed) {
                sweepPower = 1;
                sweepPressed = true;
            } else if (controller1.dpad_up() && !sweepPressed) {
                sweepPower = -1;
                sweepPressed = true;
            } else if (controller1.dpad_left() && sweepPressed) {
                sweepPower = 0;
                sweepPressed = false;
            }
            robot.collector.setPower(sweepPower);

            if (controller2.right_bumper()) {
                shooterPower = .8;
            } else if (controller2.left_bumper()) {
                shooterPower = 0;
            }
            robot.shooter.setPower(-shooterPower);
        }

    }
    public void waitNow(long waitTime) {
        try {
            Thread.sleep(waitTime);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

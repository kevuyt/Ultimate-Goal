package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import Library4997.MasqWrappers.MasqLinearOpMode;
import Library4997.MasqSensors.MasqClock;

/**
 * TeleOp NFS
 */
@TeleOp(name="ONENFS", group="Final")
public class OneNFS extends MasqLinearOpMode implements Constants {
    public MasqClock clock = new MasqClock();
    @Override
    public void runLinearOpMode() throws InterruptedException {
        while (!isStarted()) {
            robot.indexer.setPosition(0);
            dash.create("COUNT");
            dash.update();
        }
        double power = 0;
        waitForStart();
        robot.shooter.runUsingEncoder();
        while (opModeIsActive()) {
            robot.shooter.killRate(opModeIsActive());
            float move = -controller1.left_stick_y();
            float turn = -controller1.right_stick_x();
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
            if (left != 0 || right != 0) {
                robot.lights.setPower(1);
            }
            else {
                robot.lights.setPower(0);
            }
            robot.move(controller1.right_bumper(), controller1.left_bumper(), robot.collector, COLLECTOR_IN);

            if(controller1.right_trigger() > 0) {
                power += REV_UP;
                if (power > TARGET_POWER) {
                    power = TARGET_POWER;
                }
                robot.shooter.setPower(power);
            }
            else {
                power -= REV_DOWN;
                if (power < 0) {
                    power = 0;
                }
                robot.shooter.setPower(power);
            }
            if (controller1.x() && robot.shooter.getPower() < (TARGET_POWER + SHOOTER_ERROR)) {
                robot.indexer.setPosition(INDEXER_OPENED);
            }

            else {
                robot.indexer.setPosition(INDEXER_CLOSED);
            }
            dash.create("Shooter Power", robot.shooter.getRate());
            dash.create("Left Power", left);
            dash.create("Right Power", right);
            dash.create("Voltage", robot.getBatteryVoltage());
            dash.update();
        }
    }



}
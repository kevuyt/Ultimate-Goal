package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import Library4997.MasqLinearOpMode;

/**
 * TeleOp NFS
 */
@TeleOp(name="TeleOpNFS", group="Final")

public class NFS extends MasqLinearOpMode implements Constants {
    public void main() throws InterruptedException {

    }
    @Override
    public void runLinearOpMode() throws InterruptedException {
        int i = 0;
        while (!isStarted()) {
            robot.indexer.setPosition(0);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.update();
        }
        double power = 0;
        waitForStart();
        robot.shooter.runUsingEncoder();
        while (opModeIsActive()) {
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


            if (controller1.right_bumper()) {
                robot.collector.setPower(COLLECTOR_IN);
            }
            else if (controller1.left_bumper()) {
                robot.collector.setPower(COLLECTOR_OUT);
            }
            else {
                robot.collector.setPower(0);
            }

            if(controller2.right_bumper()) {
                power += REV_UP;
                if (power > TARGET_POWER) {
                    power = TARGET_POWER;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                robot.shooter.setPower(power);
            }

            else if(controller2.left_bumper()) {
                power += REV_UP;
                if (power > TARGET_POWER) {
                    power = TARGET_POWER + LOW_POWER_FACTOR;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                robot.shooter.setPower(power);
            }
            else {
                power -= REV_DOWN;
                if (power < TARGET_POWER) {
                    telemetry.addLine("Shooter is Not Revved Up.");
                }
                if (power < 0) {
                    power = 0;
                }
                robot.shooter.setPower(power);
            }

            if (controller2.right_bumper() && controller2.x() && robot.shooter.getPower() < (TARGET_POWER + SHOOTER_ERROR)) {
                robot.indexer.setPosition(INDEXER_OPENED);
            }

            else if (controller2.left_bumper() && controller2.x() && robot.shooter.getPower() < (TARGET_POWER + LOW_POWER_FACTOR + SHOOTER_ERROR)) {
                robot.indexer.setPosition(INDEXER_OPENED);
            }

            else {
                robot.indexer.setPosition(INDEXER_CLOSED);
            }

            if (controller2.dpad_left()){
                robot.rightPresser.setPower(BEACON_OUT);
            }

            else if (controller2.dpad_right()){
                robot.rightPresser.setPower(BEACON_IN);
            }

            else {
                robot.rightPresser.setPower(0);
            }

            if (controller2.dpad_up()){
                robot.leftPresser.setPower(BEACON_OUT);
            }
            else if (controller2.dpad_down()){
                robot.leftPresser.setPower(BEACON_IN);
            }
            else {
                robot.leftPresser.setPower(0);
            }

            telemetry.addData("Shooter Power", robot.shooter.getPower());
            telemetry.addData("Left Power", left);
            telemetry.addData("Right Power", right);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.addData("Rate", robot.shooter.getRate());
            telemetry.update();
        }
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


}
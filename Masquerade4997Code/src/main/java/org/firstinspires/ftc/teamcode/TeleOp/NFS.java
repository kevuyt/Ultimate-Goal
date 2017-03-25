package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import Library4997.MasqLinearOpMode;

/**
 * TeleOp NFS
 */
@TeleOp(name="TeleOpNFS", group="Final")// change name

public class NFS extends MasqLinearOpMode { // change file name
    public void main() throws InterruptedException {

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


    @Override
    public void runLinearOpMode() throws InterruptedException {
        while (!isStarted()) {
            robot.indexer.setPosition(0);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.update();
            idle();
        }

        waitForStart();
        robot.shooter.runUsingEncoder();
        double power = 0;
        double revUpFactor = 0.1;
        double revDownFactor = 0.01;
        double targetPower = -0.8;
        double lowPowerFactor = 0.2;
        while (opModeIsActive()) {
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double collector = -1.5;
            double left = move - turn;
            double right = move + turn;

            if (gamepad1.right_trigger > 0) {
                robot.lift.setPower(1);
            }
            else if (gamepad1.left_trigger >0){
                robot.lift.setPower(-1);
            }
            else {
                robot.lift.setPower(0);
            }
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


            if (gamepad1.right_bumper) {
                robot.collector.setPower(collector);
            }
            else if (gamepad1.left_bumper) {
                robot.collector.setPower(-collector);
            }
            else {
                robot.collector.setPower(0);
            }

            if(gamepad2.right_bumper) {
                power += revUpFactor;
                if (power > targetPower) {
                    power = targetPower;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                robot.shooter.setPower(power);
            }

            else if(gamepad2.left_bumper) {
                power += revUpFactor;
                if (power > targetPower) {
                    power = targetPower + lowPowerFactor;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                robot.shooter.setPower(power);
            }
            else {
                power -= revDownFactor;
                if (power < targetPower) {
                    telemetry.addLine("Shooter is Not Revved Up.");
                }
                if (power < 0) {
                    power = 0;
                }
                robot.shooter.setPower(power);
            }

            if (gamepad2.right_bumper && gamepad2.x && robot.shooter.getPower() < (targetPower + 0.05)) {
                robot.indexer.setPosition(0.6);
            }

            else if (gamepad2.left_bumper && gamepad2.x && robot.shooter.getPower() < (targetPower + lowPowerFactor + 0.05)) {
                robot.indexer.setPosition(0.6);
            }

            else {
                robot.indexer.setPosition(0);
            }

            if (gamepad2.dpad_left){
                robot.rightPresser.setPower(1);
            }

            else if (gamepad2.dpad_right){
                robot.rightPresser.setPower(-1);
            }

            else {
                robot.rightPresser.setPower(0);
            }
            if (gamepad2.dpad_up){
                robot.leftPresser.setPower(1);
            }

            else if (gamepad2.dpad_down){
                robot.leftPresser.setPower(-1);
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
}
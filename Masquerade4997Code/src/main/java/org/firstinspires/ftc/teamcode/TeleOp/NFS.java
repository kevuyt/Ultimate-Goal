package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import Library4997.MasqWrappers.MasqLinearOpMode;
import Library4997.MasqSensors.MasqClock;

/**
 * TeleOp NFS
 */
@TeleOp(name="TeleOpNFS", group="Final")

public class NFS extends MasqLinearOpMode implements Constants {
    public MasqClock clock = new MasqClock();
    public void main() throws InterruptedException {

    }
    @Override
    public void runLinearOpMode() throws InterruptedException {

        int i = 0;
        double count = 0;
        while (!isStarted()) {
            robot.indexer.setPosition(0);
            dash.create("COUNT", count);
            if(controller1.apr()) {
                count +=1;
            }
            dash.update();
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
            robot.move(robot.lights, Math.cos( (4 * Math.PI * clock.milliseconds()) + Math.PI ) + 1);
            robot.move(controller1.right_bumper(), controller1.left_bumper(), robot.collector, COLLECTOR_IN);

            if(controller2.right_bumper()) {
                power += REV_UP;
                if (power > TARGET_POWER) {
                    power = TARGET_POWER;
                }
                robot.shooter.setPower(power);
            }
            else if(controller2.left_bumper()) {
                power += REV_UP;
                if (power > TARGET_POWER) {
                    power = TARGET_POWER + LOW_POWER_FACTOR;
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
            robot.move((controller2.right_bumper() && controller2.x() && robot.shooter.getPower() < (TARGET_POWER + SHOOTER_ERROR)),
                    controller2.left_bumper() && controller2.x() && robot.shooter.getPower() < TARGET_POWER + LOW_POWER_FACTOR + SHOOTER_ERROR,
                    robot.indexer, INDEXER_OPENED, INDEXER_CLOSED);

            robot.move(controller2.dpad_left(), controller2.dpad_right(), robot.rightPresser, BEACON_IN);
            robot.move(controller2.dpad_up(), controller2.dpad_down(), robot.leftPresser, BEACON_IN);
            
            dash.create("Shooter Power", robot.shooter.getPower());
            dash.create("Left Power", left);
            dash.create("Right Power", right);
            dash.create("Voltage", getBatteryVoltage());
            dash.update();
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
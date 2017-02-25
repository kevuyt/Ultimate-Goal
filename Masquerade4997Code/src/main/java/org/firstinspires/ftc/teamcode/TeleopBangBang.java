package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import BasicLib4997.MasqMotors.TankDrive.MasqRobot;
/**
 * TeleOp NFS
 */
@TeleOp(name="TeleOpNFSBB", group="Final")// change name

public class TeleopBangBang extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }
    MasqRobot chimera = new MasqRobot(telemetry);
    double power = 0;
    double revUpFactor = 0.1;
    double revDownFactor = 0.01;
    double targetPower = -0.6;
    double lowPowerFactor = 0;
    double TARGET_VOLTAGE = 12.2;
    double voltage = 0;
    double kP = -0.18;
    boolean IS_GATE_OPEN = false;
    double fTarget = 7.5e-7;
    double fVelocity = 0.0;


    int fEncoder = 0;
    int fLastEncoder = 0;

    long fVelocityTime = 0;
    long fLastVelocityTime = 0;

    double tolerance = 0.5e-7;
    public void runOpMode() throws InterruptedException {

        while (!isStarted()) {
            chimera.setIndexer(0);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.update();
            idle();
        }

        waitForStart();
        while (opModeIsActive()) {
            float turn = -gamepad1.left_stick_y;
            float move = -gamepad1.right_stick_x;
            double collector = -1.5;
            double left = move - turn;
            double right = move + turn;

            if (gamepad1.right_trigger > 0) {
                left /= 3;
                right /= 3;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }
            if (left > 1.0) {
                left /= left;
                right /= left;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            } else if (right > 1.0) {
                left /= right;
                right /= right;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            } else {
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }


            if (gamepad1.right_bumper) {
                chimera.setPowerCollector(collector);
            } else if (gamepad1.left_bumper) {
                chimera.setPowerCollector(-collector);
            } else {
                chimera.setPowerCollector(0);
            }

            if (gamepad2.right_bumper) {
                voltage = batteryVoltage();
                double error = TARGET_VOLTAGE - voltage;
                double motorOut = (error * kP) + .72;
                if (motorOut < -1){
                    motorOut = -1;
                }
                chimera.setPowerShooter(-motorOut);
            }
            else if (gamepad2.left_bumper) {
                chimera.shooter.setPower(-1);
            }
            else {
                chimera.setPowerShooter(0);
            }

            if (gamepad2.right_bumper && gamepad2.x) {
                chimera.setIndexer(0.6);
                chimera.sleep(900);
                chimera.setIndexer(0);
                chimera.sleep(1500);
            } else if (gamepad2.left_bumper && gamepad2.x) {
                chimera.setIndexer(0.6);
                chimera.sleep(900);
                chimera.setIndexer(0);
                chimera.sleep(1500);
            } else {
                chimera.setIndexer(0);
            }

            if (gamepad2.dpad_left) {
                chimera.setRightPresser(1);
            } else if (gamepad2.dpad_right) {
                chimera.setRightPresser(-1);
            } else {
                chimera.setRightPresser(0);
            }
            if (gamepad2.dpad_up) {
                chimera.setLeftPresser(1);
            } else if (gamepad2.dpad_down) {
                chimera.setLeftPresser(-1);
            } else {
                chimera.setLeftPresser(0);
            }

            telemetry.addData("Shooter Power", chimera.shooter.getPower());
            telemetry.addData("Left Power", left);
            telemetry.addData("Right Power", right);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.addData("Rate", chimera.shooter.getRate());
            telemetry.update();
        }

    }
     double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    public void setFPower(double power)
    {
        chimera.setPowerShooter(power);
    }

    public void bangBang() {
        fVelocityTime = System.nanoTime();
        fEncoder = (int)chimera.shooter.getCurrentPos();
        fVelocity = (double)(fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if(fVelocity >= (fTarget + tolerance))
        {
            setFPower(0.8);
        }

        else if(fVelocity < (fTarget - tolerance))
        {
            setFPower(0.8);
        }

        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }
    public double batteryVoltage()
    {
        return this.hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void voltageshoot() {

    }



}
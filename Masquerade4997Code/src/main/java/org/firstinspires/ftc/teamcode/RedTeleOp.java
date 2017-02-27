package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;
/**
 * TeleOp NFS
 */
@TeleOp(name="RedTeleOp", group="Final")// change name

public class RedTeleOp extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {

        MasqRobot chimera = new MasqRobot(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0);
            telemetry.update();
            idle();
        }

        waitForStart();
        double power = 0;
        double revUpFactor = 0.1;
        double revDownFactor = 0.01;
        double targetPower = -1;
        double lowPowerFactor = 0.1;
        while (opModeIsActive()) {
            float move = -gamepad1.left_stick_y;
            float turn = -gamepad1.right_stick_x;
            double collector = -1.5;
            double left = move - turn;
            double right = move + turn;

            if (gamepad1.a) {
                left/= 3;
                right /=3;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }
            if(left > 1.0) {
                left /= left;
                right /= left;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }

            else if (right > 1.0) {
                left /= right;
                right /= right;
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }

            else {
                chimera.setPowerLeft(-left);
                chimera.setPowerRight(-right);
            }


            if (gamepad1.right_bumper) {
                chimera.setPowerCollector(collector);
            }
            else if (gamepad1.left_bumper) {
                chimera.setPowerCollector(-collector);
            }
            else {
                chimera.setPowerCollector(0);
            }

            if(gamepad2.right_bumper) {
                power += revUpFactor;
                if (power > targetPower) {
                    power = targetPower;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                chimera.setPowerShooter(targetPower);
            }

            else if(gamepad2.left_bumper) {
                power += revUpFactor;
                if (power > targetPower) {
                    power = targetPower + lowPowerFactor;
                    telemetry.addLine("Shooter is Revved Up.");
                }
                chimera.shooter.setPower(targetPower);
            }
            else {
                power -= revDownFactor;
                if (power < targetPower) {
                    telemetry.addLine("Shooter is Not Revved Up.");
                }
                if (power < 0) {
                    power = 0;
                }
                chimera.setPowerShooter(power);
            }

            if (gamepad2.right_bumper && gamepad2.x && chimera.shooter.getPower() < targetPower + 0.05) {
                chimera.setIndexer(0.6);
            }

            else if (gamepad2.left_bumper && gamepad2.x && chimera.shooter.getPower() < targetPower + 0.05) {
                chimera.setIndexer(0.6);
            }

            else {
                chimera.setIndexer(0);
            }

            if (gamepad2.a){
                chimera.setRightPresser(1);
            }

            else if (gamepad2.y){
                chimera.setRightPresser(-1);
            }

            else {
                chimera.setRightPresser(0);
            }

            telemetry.addData("Shooter Power", chimera.shooter.getPower());
            telemetry.addData("Left Power", left);
            telemetry.addData("Right Power", right);
            telemetry.addData("Rate", chimera.shooter.getRate());
            telemetry.update();
        }

    }


}
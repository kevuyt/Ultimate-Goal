package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 11/5/16.
 */
@TeleOp(name="TeleopTank", group="Final")// change name
public class TankTeleOp extends LinearOpMode{
    public void main() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double LOW_POWER = 0.50;
        double POWER = 0.70;
        double HIGH_POWER = 0.90;
        MasqRobot chimera = new MasqRobot(telemetry);

        while (!isStarted()) {

            chimera.driveTrain.telemetryRun();
            chimera.imu.telemetryRun();
            telemetry.update();
            idle();

        }

        waitForStart();
        double  power   = 0;
        double shooterRevDownFactor = 0.001;
        double shooterRevUpFactor = 0.1;
        while (opModeIsActive()) {
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;
            double collector = -1.5;
            double MAX_SHOOTER_POWER = 1;

            chimera.setPowerLeft(left);
            chimera.setPowerRight(right);

            if (gamepad1.right_bumper) {
                chimera.setPowerCollector(collector);
            } else if (gamepad1.left_bumper) {
                chimera.setPowerCollector(-collector);
            } else {
                chimera.setPowerCollector(0);
            }

            if (gamepad2.right_bumper) {
                power += shooterRevUpFactor;
                if (power >= 1) {
                    power = MAX_SHOOTER_POWER;
                    telemetry.addLine("shooter is revved up");
                }
                chimera.setPowerShooter(power);
            } else {
                power -= shooterRevDownFactor;
                if (power <= 0) {
                    power = 0;
                    telemetry.addLine("shooter powered down");
                }
                chimera.setPowerShooter(power);
            }

            //indexer


            //beaconPresser
            if (gamepad2.b) {
                chimera.setRightPresser(1);

            } else if (gamepad2.y) {
                chimera.setRightPresser(-1);
            } else {
                chimera.setRightPresser(0);
            }

            telemetry.update();
            idle();
        }
    }
}

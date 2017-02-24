package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.MasqMotors.TankDrive.Direction;
import BasicLib4997.MasqMotors.TankDrive.MasqRobot;
/**
 * TeleOp NFS
 */
@Autonomous(name="TestMeme", group="Final")// change name

public class Test extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }
    @Override
    public void runOpMode() throws InterruptedException {

        MasqRobot chimera = new MasqRobot(telemetry);
        while (!isStarted()) {
            double rate = (chimera.shooter.getRate() + chimera.shooter2.getRate())/2;
            telemetry.addData("RATE", rate);
            chimera.setIndexer(0);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.update();
            idle();
        }

        waitForStart();
        double power = -0.5;
        chimera.setPowerCollector(-1);
        chimera.setPowerShooter(power);
        chimera.drive(0.5, 15, Direction.FORWARD);
        double i  = 0.001;
        while ((((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) > - 500 && ((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) > -525)) {
            chimera.setIndexer(0);
            telemetry.addData("RATE", (chimera.shooter.getRate() + chimera.shooter2.getRate())/2);
            chimera.setPowerShooter(power - i);
            i += 0.001;
            telemetry.update();
        }
        chimera.setIndexer(0.6);
        chimera.sleep(500);
        chimera.setIndexer(0);
        chimera.sleep(1000);
        i = 0.001;
        while ((((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) > - 500 && ((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) > -525)) {
        chimera.setIndexer(0);
        telemetry.addData("RATE", (chimera.shooter.getRate() + chimera.shooter2.getRate())/2);
        chimera.setPowerShooter(power - i);
        i += 0.001;
        telemetry.update();
        }
        chimera.setIndexer(0.6);
        chimera.sleep(700);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);
        chimera.drive(0.5, 15, Direction.BACKWARD);

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
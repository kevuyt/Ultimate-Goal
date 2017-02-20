package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;
/**
 * TeleOp NFS
 */
@TeleOp(name="Test", group="Final")// change name

public class Test extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }
    @Override
    public void runOpMode() throws InterruptedException {

        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setPowerShooter(-0.75);
            if ((((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) < 500)) {
                telemetry.addLine("less than");
            }
            else {
                telemetry.addLine("greater than");
            }
            double rate = (chimera.shooter.getRate() + chimera.shooter2.getRate())/2;
            telemetry.addData("RATE", rate);
            chimera.setIndexer(0);
            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.update();
            idle();
        }

        waitForStart();
        chimera.setPowerCollector(-1);
        chimera.setPowerShooter(-0.75);
        chimera.drivePID(0.5, 15, Direction.FORWARD);
        while ((((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) < 500)) {
            chimera.setIndexer(0);
        }
        chimera.setIndexer(0.6);
        chimera.sleep(700);
        while ((((chimera.shooter.getRate() + chimera.shooter2.getRate())/2) < 500)) {
            chimera.setIndexer(0);
        }
        chimera.setIndexer(0.6);
        chimera.sleep(700);
        chimera.sleep(1000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);

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
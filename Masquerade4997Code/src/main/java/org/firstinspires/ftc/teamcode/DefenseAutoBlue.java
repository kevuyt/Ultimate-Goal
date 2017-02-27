package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.MasqMotors.MasqRobot.Direction;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 10/6/16.
 */

@Autonomous(name = "DEFENSE_AUTO_BLUE", group = "Test") // change name
public class DefenseAutoBlue extends LinearOpMode { // change file name
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
            chimera.runAllTelemetry();
            telemetry.update();
            idle();
        }
        waitForStart();
        chimera.setPowerCollector(-1);
        chimera.setPowerShooter(newShooterPower(-0.65));
        chimera.drive(POWER, 10, Direction.FORWARD);
        chimera.sleep(1000);
        chimera.setIndexer(0.6);
        chimera.sleep(400);
        chimera.setIndexer(0);
        chimera.sleep(2000);
        chimera.setIndexer(0.6);
        chimera.sleep(1000);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.sleep(1000);
        chimera.drive(0.6, 310, Direction.FORWARD);
        chimera.turn(POWER, 43, Direction.LEFT, 3);
        chimera.sleep(3000);
        chimera.drive(1, 500, Direction.FORWARD);
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
    private double newShooterPower(double targetPower){
        double voltsge = getBatteryVoltage();
        double targetVoltage = 13.5;
        double error = targetVoltage - voltsge;
        return targetPower - (error * 0.02);
    }

}
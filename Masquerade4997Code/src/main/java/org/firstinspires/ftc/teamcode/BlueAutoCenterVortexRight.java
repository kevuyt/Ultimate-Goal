package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.Motors.TankDrive.Direction;
import BasicLib4997.Motors.TankDrive.TankDrive;

/**
 * Beacons + Shoot for Red8
 */

@Autonomous(name = "BlueAutoCenterVortexRight", group = "G1") // change name

public class BlueAutoCenterVortexRight extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double POWER = 0.50;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            chimera.setIndexer(0);
            chimera.imu.telemetryRun();
            chimera.leftColor.telemetryRun();
            chimera.rangeSensor.telemetryRun();
            telemetry.update();
            idle();
        }
        waitForStart();
        //SetPower to the shooter and drive foreword in orer to make a shot
        chimera.setPowerShooter(newShooterPower(-0.75));
        double parallelAngle = chimera.imu.getHeading();
        chimera.drivePID(POWER, 10, Direction.FORWARD);
        // find how arr of we are from our orignial position
        double disruption = chimera.imu.getHeading();
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
        //turn away from the center vortex 48 degrees - our disruption
        chimera.turnPID(POWER, (int) (48 - disruption), Direction.LEFT, 3);
        // drive parallel to the ramp
        chimera.drivePID(0.7, 320, Direction.FORWARD);
        // turn parallel to the Beacon
        double changeTurn = chimera.imu.getHeading();
        double turn = changeTurn - parallelAngle;

        chimera.turnPID(POWER, (int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        chimera.stopBlue(0.4, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 15, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(2000);
        chimera.setRightPresser(1);
        // drive to the next beacon
        chimera.drivePID(0.7, 60, Direction.FORWARD, 1000, parallelAngle);

        chimera.setRightPresser(0);

        chimera.stopBlue(0.4, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 15, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(2000);
        chimera.setRightPresser(1);








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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import BasicLib4997.MasqMotors.TankDrive.Constants;
import BasicLib4997.MasqMotors.TankDrive.Direction;
import BasicLib4997.MasqMotors.TankDrive.MasqRobot;

/**
 * Beacons + Shoot for Red8
 */

@Autonomous(name = "BlueAutoCenterVortexRight", group = "G1") // change name

public class BlueAutoCenterVortexRight extends LinearOpMode implements Constants  { // change file name
    public void main() throws InterruptedException {

    }



    @Override
    public void runOpMode() throws InterruptedException {
        boolean telemetrizeModules;
        double POWER = 0.30;
        MasqRobot chimera = new MasqRobot(telemetry);
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
        chimera.colorRejection.setActiveMode();
        chimera.rightColor.setPassiveMode();
        boolean isNeccesary = true;
        //SetPower to the shooter and drive foreword in order to make a shot
        chimera.setPowerCollector(-1);
        double power = shooterPowerAuto;
        chimera.setPowerShooter(power);
        double parallelAngle = chimera.imu.getHeading();
        chimera.drive(POWER, 15, Direction.FORWARD);
        // find how arr of we are from our orignial position
        double disruption = chimera.imu.getHeading();
        chimera.setIndexer(0.6);
        chimera.sleep(500);
        chimera.setIndexer(0);
        chimera.sleep(2000);
        chimera.setIndexer(0.6);
        chimera.sleep(700);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);
        //turn away from the center vortex 48 degrees - our disruption
        chimera.turn(POWER, (int) (49 - disruption), Direction.RIGHT, 3);
        if (chimera.angleLeftCover >= 0.5 && chimera.angleLeftCover <= 2) {
            isNeccesary = false;
        }
        // drive parallel to the ramp
        chimera.drive(0.5, 298, Direction.FORWARD);
        // turn parallel to the Beacon
        double changeTurn = chimera.imu.getHeading();
        double turn = changeTurn + parallelAngle;
        chimera.turn(POWER, (int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        //chimera.stopWhite(0.2, Direction.BACKWARD);
        chimera.stopBlueRight(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drive(POWER, 33, Direction.BACKWARD);
        // move the beacon presser
        chimera.setLeftPresser(-1);
        chimera.sleep(1000);
        chimera.setLeftPresser(1);
        double turn2 = parallelAngle + chimera.imu.getHeading();
        chimera.turn(POWER, (int) turn2, Direction.RIGHT, 2, 0.3);
        // drive to the next beacon
        double turn3 = parallelAngle + chimera.imu.getHeading();
        chimera.drive(0.5, 150, Direction.FORWARD, 500);
        chimera.turn(POWER, (int) turn3, Direction.RIGHT, 2, 0.1);
        chimera.setLeftPresser(0);
        //chimera.stopWhite(0.3, Direction.BACKWARD);
        chimera.stopBlueRight(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drive(POWER, 15, Direction.BACKWARD);
        // move the beacon presser
        chimera.setLeftPresser(-1);
        chimera.sleep(1000);
        chimera.setLeftPresser(1);








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
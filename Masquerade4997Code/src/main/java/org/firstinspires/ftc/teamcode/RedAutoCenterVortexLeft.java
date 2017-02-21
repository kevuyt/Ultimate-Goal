package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.TankDrive.Direction;
import BasicLib4997.MasqMotors.TankDrive.TankDrive;

/**
 * Beacons + Shoot for Red8
 */

@Autonomous(name = "RedAutoCenterVortexLeft", group = "G1") // change name

public class RedAutoCenterVortexLeft extends LinearOpMode { // change file name
    public void main() throws InterruptedException {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        double POWER = 0.50;
        TankDrive chimera = new TankDrive(telemetry);
        while (!isStarted()) {
            telemetry.addData("RATE", (chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2);
            telemetry.addData("SHOOTER2", (chimera.shooter2.getRate()));
            telemetry.addData("SHOOTER1", (chimera.shooter.getRate()));

            chimera.setIndexer(0);
            chimera.imu.telemetryRun();
            chimera.leftColor.telemetryRun();
            telemetry.update();
        }
        waitForStart();
        chimera.colorRejection.setActiveMode();
        chimera.rightColor.setPassiveMode();
        boolean isNeccesary = true;
        //SetPower to the shooter and drive foreword in order to make a shot
        chimera.setPowerCollector(-1);
        double power = -0.45;
        chimera.setPowerShooter(power);
        double parallelAngle = chimera.imu.getHeading();
        chimera.drivePID(POWER, 18, Direction.FORWARD);
        // find how arr of we are from our orignial position
        double disruption = chimera.imu.getHeading();
        double i = 0.001;
        telemetry.addLine("Before");
        update();
        TankDrive.getTelemetry().addTelemetry("RATE",  (chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2);
        while (!(((chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2) >= -1000) || !(((chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2) <= -1050)) {
            chimera.setIndexer(0);
            telemetry.addData("RATE", (chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2);
            chimera.setPowerShooter(power - i);
           // i += 0.001;
            update();
        }
        telemetry.addLine("after");
        update();
        chimera.setIndexer(0.6);
        chimera.sleep(500);
        chimera.setIndexer(0);
        chimera.sleep(1000);
        i = 0.001;
        while (!(((chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2) >= -1000) || !(((chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2) <= -1050)) {
            chimera.setIndexer(0);
            telemetry.addData("RATE", (chimera.shooter.getRate() + chimera.shooter2.getRate()) / 2);
            chimera.setPowerShooter(power - i);
            i += 0.001;
            update();
        }
        chimera.setIndexer(0.6);
        chimera.sleep(700);
        chimera.setPowerShooter(-0.5);
        chimera.setPowerShooter(-0.3);
        chimera.setPowerShooter(0);
        chimera.setPowerCollector(0);
        //turn away from the center vortex 48 degrees - our disruption
        chimera.turnPID(POWER, (int) (49 - disruption), Direction.LEFT, 3);
        if (chimera.angleLeftCover >= 0.5 && chimera.angleLeftCover <= 2) {
            isNeccesary = false;
        }
        // drive parallel to the ramp
        chimera.drivePID(0.5, 298, Direction.FORWARD);
        // turn parallel to the Beacon
        double changeTurn = chimera.imu.getHeading();
        double turn = changeTurn - parallelAngle;
        chimera.turnPID(POWER, (int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        chimera.stopWhite(0.2, Direction.BACKWARD);
        chimera.stopRed(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 33, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(1000);
        chimera.setRightPresser(1);
        double turn2 = parallelAngle - chimera.imu.getHeading();
        chimera.turnPID(POWER, (int) turn2, Direction.LEFT, 2, 0.3);
        if (isNeccesary) {
            chimera.turnPID(POWER, 3, Direction.LEFT, 1, 0.3);
        }
        // drive to the next beacon
        double turn3 = parallelAngle - chimera.imu.getHeading();
        chimera.drivePID(0.5, 150, Direction.FORWARD, 500);
        chimera.turnPID(POWER, (int) turn3, Direction.LEFT, 2, 0.1);
        chimera.setRightPresser(0);
        chimera.stopWhite(0.3, Direction.BACKWARD);
        chimera.stopRed(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drivePID(POWER, 25, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(1000);
        chimera.setRightPresser(1);
    }

    public void update() {
        telemetry.update();
    }
}
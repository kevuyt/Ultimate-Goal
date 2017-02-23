package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import BasicLib4997.MasqMotors.TankDrive.Direction;
import BasicLib4997.MasqMotors.TankDrive.MasqRobot;

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
        MasqRobot chimera = new MasqRobot(telemetry);
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
        chimera.turn(POWER, (int) (49 - disruption), Direction.LEFT, 3);
        if (chimera.angleLeftCover >= 0.5 && chimera.angleLeftCover <= 2) {
            isNeccesary = false;
        }
        // drive parallel to the ramp
        chimera.drive(0.5, 298, Direction.FORWARD);
        // turn parallel to the Beacon
        double changeTurn = chimera.imu.getHeading();
        double turn = changeTurn - parallelAngle;
        chimera.turn(POWER, (int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        chimera.stopWhite(0.2, Direction.BACKWARD);
        chimera.stopRed(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drive(POWER, 33, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(1000);
        chimera.setRightPresser(1);
        double turn2 = parallelAngle - chimera.imu.getHeading();
        chimera.turn(POWER, (int) turn2, Direction.LEFT, 2, 0.3);
        if (isNeccesary) {
            chimera.turn(POWER, 3, Direction.LEFT, 1, 0.3);
        }
        // drive to the next beacon
        double turn3 = parallelAngle - chimera.imu.getHeading();
        chimera.drive(0.5, 150, Direction.FORWARD, 500);
        chimera.turn(POWER, (int) turn3, Direction.LEFT, 2, 0.1);
        chimera.setRightPresser(0);
        chimera.stopWhite(0.3, Direction.BACKWARD);
        chimera.stopRed(0.2, Direction.BACKWARD);
        // drive in position to hit the beacon
        chimera.drive(POWER, 25, Direction.BACKWARD);
        // move the beacon presser
        chimera.setRightPresser(-1);
        chimera.sleep(1000);
        chimera.setRightPresser(1);
    }

    public void update() {
        telemetry.update();
    }
}
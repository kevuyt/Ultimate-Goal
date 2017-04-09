package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqLinearOpMode;
import Library4997.MasqRobot.Direction;

/**
 * Red Autonomous
 */

@Autonomous(name = "RedAutoCenterVortexLeft", group = "REd")
public class RedAutoCenterVortexLeft extends MasqLinearOpMode implements Constants {
    private int delay = 0;
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create("Press A on gamepad1 to set a delay of one second press b to reset");
            if (controller1.apr()){
                delay += 1000;
            }
            else if (controller1.bpr()){
                delay = 0;
            }
            dash.create("DELAY: ", delay);
            dash.create(robot.imu);
            dash.createSticky(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.colorRejection.setActiveMode();
        robot.rightColor.setPassiveMode();
        boolean isNeccesary = true;
        //SetPower to the shooter and drive foreword in order to make a shot
        robot.collector.setPower(-1);
        double power = 0.5;
        robot.shooter.setPower(power);
        double parallelAngle = robot.imu.getHeading();
        robot.drive(15);
        // find how arr of we are from our orignial position
        double disruption = robot.imu.getHeading();
        robot.indexer.setPosition(0.6);
        robot.sleep(500);
        robot.indexer.setPosition(0);
        robot.sleep(2000);
        robot.indexer.setPosition(0.6);
        robot.sleep(700);
        robot.shooter.setPower(-0.5);
        robot.shooter.setPower(-0.3);
        robot.shooter.setPower(0);
        robot.collector.setPower(0);
        //turn away from the center vortex 48 degrees - our disruption
        robot.turn((int) (49 - disruption), Direction.LEFT, 3);
        if (robot.angleLeftCover >= 0.5 && robot.angleLeftCover <= 2) {
            isNeccesary = false;
        }
        // drive parallel to the ramp
        robot.drive(290);
        // turn parallel to the Beacon
        double changeTurn = robot.imu.getHeading();
        double turn = changeTurn - parallelAngle;
        robot.turn((int) turn, Direction.RIGHT, 5);
        // Stop at first Beacon
        //robot
        //.stopWhite(0.2, Direction.BACKWARD);
        robot.stopRed(robot.leftColor, POWER_LOW, Direction.BACKWARD);
        // drive in position to hit the beacon
        robot.drive(33);
        // move the beacon presser
        robot.rightPresser.setPower(-1);
        robot.sleep(1000);
        robot.rightPresser.setPower(1);
        double turn2 = parallelAngle - robot.imu.getHeading();
        robot.turn((int) turn2, Direction.LEFT);
        if (isNeccesary) {
            robot.turn(3, Direction.LEFT);
        }
        // drive to the next beacon
        double turn3 = parallelAngle - robot.imu.getHeading();
        robot.drive(150);
        robot.turn((int) turn3, Direction.LEFT);
        robot.rightPresser.setPower(0);
        //robot
        //.stopWhite(0.3, Direction.BACKWARD);
        robot.stopRed(robot.leftColor,POWER_LOW, Direction.BACKWARD);
        // drive in position to hit the beacon
        robot.drive(25);
        // move the beacon presser
        robot.rightPresser.setPower(-1);
        robot.sleep(1000);
        robot.rightPresser.setPower(1);
    }

    public void update() {
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;

/**
 * Created by Archish on 2/25/18.
 */
@Autonomous(name = "RedAutoV3", group = "Autonomus")
public class RedAutoV3 extends MasqLinearOpMode implements Constants{
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setPosition(Flipper.Position.MID);
        while (!opModeIsActive()) {
            dash.create("Initialized");
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        String vuMark = readVuMark();
        runJewel();
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
        if (MasqUtils.VuMark.isLeft(vuMark)) leftMulti();
        else leftMulti();
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(500);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        robot.sleep(250);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(100);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        double startAngle = robot.imu.getHeading();
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(20, POWER_OPTIMAL, Direction.BACKWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(27, POWER_OPTIMAL, Direction.BACKWARD, 2);
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(15, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(28, POWER_OPTIMAL, Direction.BACKWARD, 3);
        double endAngle = robot.imu.getHeading();
        robot.turn(60 + (endAngle - startAngle), Direction.RIGHT);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
    public void leftMulti() {
        robot.turn(25, Direction.RIGHT);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.drive(55, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setPosition(Flipper.Position.MID);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.sleep(2000);
        robot.drive(5);
        robot.drive(5, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.turn(15, Direction.RIGHT);
        robot.flipper.setPosition(Flipper.Position.MID);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.sleep(1000);
        robot.drive(50, POWER_HIGH, Direction.BACKWARD);
        robot.drive(10);
    }
}

package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import MasqueradeSubSystems.SubSystems.Flipper;

/**
 * Created by Archish on 1/19/18.
 */
@Autonomous(name = "RedAuto_RP", group = "Group3")
public class RedAuto_RP extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setPosition(Flipper.Position.MID);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        String vuMark = readVuMark();
        runJewel();
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(1500);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        robot.sleep(1500);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        double startAngle = robot.imu.getHeading();
        robot.drive(20, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(18, POWER_LOW, Direction.FORWARD);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(16, POWER_OPTIMAL, Direction.BACKWARD, 2);
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(2, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD, 3);
        robot.jewelArmRed.setPosition(JEWEL_RED_HOVER);
        robot.stop(robot.jewelColorRed, .2, Direction.BACKWARD);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        double endAngle = robot.imu.getHeading();
        robot.drive(1, POWER_LOW, Direction.BACKWARD, 1);
        robot.turn(90 + (endAngle - startAngle), Direction.RIGHT);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
}
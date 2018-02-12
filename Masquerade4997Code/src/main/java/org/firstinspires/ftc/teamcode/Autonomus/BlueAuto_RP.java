package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import MasqueradeSubSystems.SubSystems.Flipper;

/**
 * Created by Archish on 1/19/18.
 */
@Autonomous(name = "BlueAuto_RP", group = "Group3")
public class BlueAuto_RP extends MasqLinearOpMode implements Constants {
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
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1000);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        robot.sleep(1000);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1000);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        double startAngle = robot.imu.getHeading();
        robot.drive(25, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(23, POWER_LOW, Direction.FORWARD);
        robot.blueRotator.setPosition(ROTATOR_BLUE_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqUtils.VuMark.isRight(vuMark)) robot.drive(22, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_HOVER);
        robot.stop(robot.jewelColorBlue, .2, Direction.BACKWARD);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1000);
        double endAngle = robot.imu.getHeading();
        robot.turn(100 - (endAngle - startAngle), Direction.LEFT);
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.sleep(1000);
        robot.drive(10, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
}

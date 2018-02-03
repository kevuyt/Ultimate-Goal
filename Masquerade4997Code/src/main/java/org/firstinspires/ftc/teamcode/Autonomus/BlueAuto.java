package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/2/17.
 */
@Autonomous(name = "BlueAuto", group = "Group1")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipLeft.setPosition(FLIPPER_MID_LEFT);
        robot.flipRight.setPosition(FLIPPER_MID_RIGHT);
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
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
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
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(22, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark)) robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_HOVER);
        robot.stop(robot.jewelColorBlue, .2, Direction.BACKWARD);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        robot.sleep(1000);
        double endAngle = robot.imu.getHeading();
        robot.turn(100 - (endAngle - startAngle), Direction.LEFT);
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipLeft.setPosition(FLIPPER_OUT_LEFT);
        robot.flipRight.setPosition(FLIPPER_OUT_RIGHT);
        robot.sleep(1000);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipLeft.setPosition(FLIPPER_DOWN_LEFT);
        robot.flipRight.setPosition(FLIPPER_DOWN_RIGHT);
    }
}
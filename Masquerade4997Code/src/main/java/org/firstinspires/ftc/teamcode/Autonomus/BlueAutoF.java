package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/27/18.
 */
@Autonomous(name = "BlueAutoF", group = "Group2")
public class BlueAutoF extends MasqLinearOpMode implements Constants {
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
    public void runVuMark (String vuMark) {
        robot.drive(25, POWER_LOW, Direction.BACKWARD);
        robot.drive(23, POWER_LOW, Direction.FORWARD);
        robot.drive(5, POWER_LOW, Direction.BACKWARD);
        if (MasqExternal.VuMark.isRight(vuMark)) robot.turn(40, Direction.RIGHT);
        else if (MasqExternal.VuMark.isCenter(vuMark)) robot.turn(30, Direction.RIGHT);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.turn(15, Direction.RIGHT);
        else robot.turn(30, Direction.RIGHT);
        robot.drive(90, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipLeft.setPosition(FLIPPER_OUT_LEFT);
        robot.flipRight.setPosition(FLIPPER_OUT_RIGHT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipLeft.setPosition(FLIPPER_DOWN_LEFT);
        robot.flipRight.setPosition(FLIPPER_DOWN_RIGHT);
    }
}
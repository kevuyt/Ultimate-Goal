package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/16/17.
 */
@Autonomous(name = "RedAutoV3", group = "Autonomus")
public class RedAutoV3 extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
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
        robot.jewelArmRed.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1500);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(COLOR_SEEN);
        else robot.redRotator.setPosition(COLOR_NOT_SEEN);
        robot.sleep(1500);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(175, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(200, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(225, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark)) robot.drive(200, POWER_OPTIMAL, Direction.FORWARD);
        robot.turn(70, Direction.RIGHT);
        robot.drive(60);
        robot.drive(60, POWER_OPTIMAL, Direction.BACKWARD);
    }
    public void runMultiGlyph() {
        robot.turn(90, Direction.LEFT, 1);
        robot.turn(90, Direction.LEFT, 1);
        robot.turn(90, Direction.RIGHT, 1);
        robot.turn(90, Direction.RIGHT, 1);
        robot.drive(90);
    }
}
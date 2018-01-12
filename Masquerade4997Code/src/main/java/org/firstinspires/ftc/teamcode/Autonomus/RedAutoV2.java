package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/11/17.
 */
@Autonomous(name = "RedAutoV2", group = "Autonomus")
@Disabled
public class RedAutoV2 extends MasqLinearOpMode implements Constants {
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
        runMultiGlyph();
    }
    public void runJewel() {
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1500);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(COLOR_SEEN);
        else robot.blueRotator.setPosition(COLOR_SEEN);
        robot.sleep(1500);
        robot.blueRotator.setPosition(ROTATOR_CENTER);
        robot.sleep(1500);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(90, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(250, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(250, POWER_OPTIMAL, Direction.FORWARD);
        robot.turn(90, Direction.RIGHT);
        robot.drive(30);
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD);
    }
    public void runMultiGlyph() {
        robot.turn(180, Direction.LEFT);
        robot.drive(90);
    }
}
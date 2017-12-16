package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/2/17.
 */
@Autonomous(name = "BlueAutoV2", group = "Autonomus")
public class BlueAutoV2 extends MasqLinearOpMode implements Constants {
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
        //runMultiGlyph();
    }
    public void runJewel() {
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1500);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(COLOR_SEEN);
        else robot.blueRotator.setPosition(COLOR_NOT_SEEN);
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
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(175, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(200, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(225, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark)) robot.drive(200, POWER_OPTIMAL, Direction.FORWARD);
        robot.turn(80, Direction.LEFT);
        robot.drive(60);
        robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_OPENED);
        robot.glyphSystemTop.setPosition(GLYPH_TOP_OPENED);
        robot.drive(60, POWER_OPTIMAL, Direction.BACKWARD);
    }
    public void runMultiGlyph() {
        robot.glyphSystemBottom.setPosition(GLYPH_BOTTOM_CLOSED);
        robot.turn(90, Direction.LEFT, 1);
        robot.turn(90, Direction.LEFT, 1);
        robot.bottomIntake.setPower(-1);
        robot.stop(robot.bottomLimit);
        robot.turn(90, Direction.RIGHT, 1);
        robot.turn(90, Direction.RIGHT, 1);
        robot.drive(90);
       /* moveToTop(true);
        robot.bottomIntake.setPower(-1);
        robot.stop(robot.bottomLimit);
        moveToTop(false);*/
    }
    public void moveToTop(boolean doTop) {
        robot.bottomIntake.setPower(0);
        robot.glyphSystemTop.setPosition(GLYPH_TOP_CLOSED);
        if (doTop)robot.glyphSystemBottom.setPosition(GLYPH_TOP_OPENED);
        robot.lift.setDistance(200);
        robot.lift.runToPosition(Direction.FORWARD, .7);
    }

}
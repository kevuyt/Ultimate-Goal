package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/10/17.
 */
@Autonomous(name = "BlueAuto", group = "Autonomus")
public class BlueAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        dash.create(INIT_MESSAGE);
        dash.update();
        waitForStart();
        robot.initializeServos();
        robot.vuforia.activateVuMark();
        robot.waitForVuMark();
        String vuMark = robot.vuforia.getVuMark();
        dash.create(vuMark);
        dash.update();
        runJewel();
        robot.driveTrain.setKp(0.01);
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
    }
    public void runJewel () {
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(2000);
        if (robot.jewelColor.isBlue()) {
            robot.turn(20, Direction.LEFT);
            robot.jewelArm.setPosition(JEWEL_IN);
            robot.turn(20, Direction.RIGHT);
        }
        else {
            robot.turn(20, Direction.RIGHT);
            robot.jewelArm.setPosition(JEWEL_IN);
            robot.turn(20, Direction.LEFT);
        }
    }

    public void runVuMark(String vuMark) {
        if (MasqExternal.VuMark.isCenter(vuMark)){
            robot.drive(70, POWER_LOW, Direction.BACKWARD);
            robot.turn(90, Direction.RIGHT);
            robot.glyphSystemTop.setPosition(GLYPH_OPENED);
            robot.glyphSystemBottom.setPosition(1);
            robot.drive(20);
        }
        else if (MasqExternal.VuMark.isLeft(vuMark)){
            robot.drive(60, POWER_LOW, Direction.BACKWARD);
            robot.turn(90, Direction.RIGHT);
            robot.glyphSystemTop.setPosition(GLYPH_OPENED);
            robot.glyphSystemBottom.setPosition(1);
            robot.drive(20);
        }
        else if (MasqExternal.VuMark.isRight(vuMark)){
            robot.drive(90, POWER_LOW, Direction.BACKWARD);
            robot.turn(90, Direction.RIGHT);
            robot.glyphSystemBottom.setPosition(1);
            robot.glyphSystemTop.setPosition(GLYPH_OPENED);
            robot.drive(20);
        }
        else {
            robot.drive(70, POWER_LOW, Direction.BACKWARD);
            robot.turn(90, Direction.RIGHT);
            robot.glyphSystemTop.setPosition(GLYPH_OPENED);
            robot.glyphSystemBottom.setPosition(1);
            robot.drive(20);
        }
        robot.drive(50, POWER_LOW, Direction.BACKWARD);
    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/6/17.
 */
@Autonomous(name = "VuMarkAuto", group = "Autonomus")
public class VuMarkAuto extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuMark(hardwareMap);
        dash.create(">>> Press Play to Begin The Op Mode.");
        dash.update();
        robot.initalizeServos();
        waitForStart();
        robot.vuforia.activateVuMark();
        while (robot.vuforia.getVuMark() == "UNKNOWN") {}
        String vuMark = robot.vuforia.getVuMark();
        switch (vuMark){
            case "LEFT" :
                robot.drive((int) DISTANCE_TO_LEFT_BOX, POWER_LOW, Direction.BACKWARD);
                robot.turn(90, Direction.RIGHT);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            case "RIGHT" :
                robot.drive((int) DISTANCE_TO_RIGHT_BOX, POWER_LOW, Direction.BACKWARD);
                robot.turn(90, Direction.RIGHT);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            case "CENTER" :
                robot.drive((int) DISTANCE_TO_CENTER_BOX, POWER_LOW, Direction.BACKWARD);
                robot.turn(90, Direction.RIGHT);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            default: break;
        }
    }
    @Override
    public void stopLinearOpMode () {
        robot.jewelArm.setPosition(JEWEL_IN);
        robot.glyphSystem.setPosition(GLYPH_OPENED);
    }

}
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
    Direction direction;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuMark(hardwareMap);
        direction = Direction.BACKWARD;
        while (!opModeIsActive()) {
            /*if (controller1.aOnPress() && direction != Direction.BACKWARD) {
                dash.clear();
                dash.create("THIS WILL GO BACKWARD");
                direction = Direction.BACKWARD;
                controller1.update();
            }
            else if (controller1.aOnPress() && direction != Direction.FORWARD) {
                dash.clear();
                dash.create("THIS WILL GO FOREWORD");
                direction = Direction.FORWARD;
                controller1.update();
            }
            controller1.update();*/
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        robot.initalizeServos();
        waitForStart();
        robot.vuforia.activateVuMark();
        while (robot.vuforia.getVuMark() == "UNKNOWN") {
            dash.create(robot.vuforia.getVuMark());
            dash.update();
        }
        String vuMark = robot.vuforia.getVuMark();
        dash.create(vuMark);
        dash.update();
        switch (vuMark){
            case "LEFT" :
                robot.drive(30, 0.5, Direction.BACKWARD);
                robot.turn(90, Direction.RIGHT);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            case "RIGHT" :
                robot.drive((int) DISTANCE_TO_RIGHT_BOX, POWER_LOW, direction);
                robot.turn(90, Direction.RIGHT);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            case "CENTER" :
                robot.drive((int) DISTANCE_TO_CENTER_BOX, POWER_LOW, direction);
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
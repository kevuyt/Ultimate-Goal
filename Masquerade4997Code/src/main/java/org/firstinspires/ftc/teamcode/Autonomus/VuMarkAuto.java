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
    boolean red;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuMark(hardwareMap);
        direction = Direction.BACKWARD;
        robot.initalizeServos();
        while (!opModeIsActive()) {
            if (controller1.aOnPress() && direction != Direction.BACKWARD && !red) {
                dash.clear();
                dash.create("THIS WILL GO BACKWARD, AND IS RED");
                red = true;
                direction = Direction.BACKWARD;
                controller1.update();
            }
            else if (controller1.aOnPress() && direction != Direction.BACKWARD && red) {
                dash.clear();
                dash.create("THIS WILL GO BACKWARD, AND IS BLUE");
                red = false;
                direction = Direction.FORWARD;
                controller1.update();
            } else if (controller1.aOnPress() && direction != Direction.FORWARD && !red) {
                dash.clear();
                dash.create("THIS WILL GO FOREWORD, AND IS RED");
                red = true;
                direction = Direction.FORWARD;
                controller1.update();
            } else if (controller1.aOnPress() && direction != Direction.FORWARD && red) {
                dash.clear();
                dash.create("THIS WILL GO FOREWORD, AND IS BLUE");
                red = false;
                direction = Direction.FORWARD;
                controller1.update();
            }
            controller1.update();
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        int addedDistance = 0;
        robot.vuforia.activateVuMark();
        robot.waitForVuMark();
        String vuMark = robot.vuforia.getVuMark();
        dash.create(vuMark);
        dash.update();
        switch (vuMark){
            case "LEFT" :
                robot.drive(80 + addedDistance, 0.5, direction);
                robot.turn(90, direction);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(20);
                break;
            case "RIGHT" :
                robot.drive((int) DISTANCE_TO_RIGHT_BOX, POWER_LOW, direction);
                robot.turn(90, direction);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            case "CENTER" :
                robot.drive((int) DISTANCE_TO_CENTER_BOX, POWER_LOW, direction);
                robot.turn(90, direction);
                robot.drive(10);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(10);
                break;
            default: break;
        }
    }
    public int runJewel () {
        int addedDistance;
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(100);
        if (red) {
            if (robot.jewelColor.isRed()) {
                robot.drive(-10);
                addedDistance = -10;
            } else {
                robot.drive(10);
                addedDistance = 10;
            }
        } else {
            if (robot.jewelColor.isBlue()) {
                robot.drive(-10);
                addedDistance = -10;
            }
            else {
                robot.drive(10);
                addedDistance = 10;
            }
        }
        return addedDistance;
    }
    @Override
    public void stopLinearOpMode () {
        robot.jewelArm.setPosition(JEWEL_IN);
        robot.glyphSystem.setPosition(GLYPH_OPENED);
    }

}
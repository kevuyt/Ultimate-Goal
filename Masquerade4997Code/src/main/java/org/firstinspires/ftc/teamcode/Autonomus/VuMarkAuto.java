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
    Direction directionTurnOne, directionTurnTwo, directionDrive;
    int distance = 10;
    boolean red;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuMark(hardwareMap);
        directionDrive = Direction.BACKWARD;
        directionTurnOne = Direction.RIGHT;
        robot.initalizeServos();
        while (!opModeIsActive()) {
            if (controller1.aOnPress() && directionDrive != Direction.BACKWARD && !red) {
                dash.clear();
                dash.create("THIS WILL GO BACKWARD, AND IS RED");
                red = true;
                directionDrive = Direction.BACKWARD;
                directionTurnOne = Direction.RIGHT;
                directionTurnTwo = Direction.RIGHT;
                controller1.update();
            }
            else if (controller1.aOnPress() && directionDrive != Direction.BACKWARD && red) {
                dash.clear();
                dash.create("THIS WILL GO BACKWARD, AND IS BLUE");
                red = false;
                directionDrive = Direction.FORWARD;
                controller1.update();
            } else if (controller1.aOnPress() && directionDrive != Direction.FORWARD && !red) {
                dash.clear();
                dash.create("THIS WILL GO FOREWORD, AND IS RED");
                red = true;
                directionDrive = Direction.FORWARD;
                controller1.update();
            } else if (controller1.aOnPress() && directionDrive != Direction.FORWARD && red) {
                dash.clear();
                dash.create("THIS WILL GO FOREWORD, AND IS BLUE");
                red = false;
                directionDrive = Direction.FORWARD;
                controller1.update();
            }
            controller1.update();
            dash.update();
        }
        waitForStart();
        int addedDistance = runJewel();
        robot.vuforia.activateVuMark();
        robot.stop(robot.vuforia, POWER_LOW, directionDrive);
        String vuMark = robot.vuforia.getVuMark();
        dash.create(vuMark);
        dash.update();
        switch (vuMark){
            case "LEFT" :
                robot.drive(80 + addedDistance, POWER_OPTIMAL, directionDrive);
                robot.turn(90, directionTurnOne);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(20);
                break;
            case "RIGHT" :
                robot.turn(30, directionTurnTwo);
                robot.drive(100, POWER_LOW, directionDrive);
                robot.turn(75, directionTurnOne);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(40);
                break;
            case "CENTER" :
                robot.drive(100, POWER_LOW, directionDrive);
                robot.turn(90, directionTurnOne);
                robot.glyphSystem.setPosition(GLYPH_OPENED);
                robot.drive(20);
                break;
            default: break;
        }
    }
    public int runJewel () {
        int addedDistance = 0;
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(5000);
        if (!red) {
            if (robot.jewelColor.isRed()) {
                robot.drive(-30);
                distance = -30;
            } else {
                robot.drive(30);
                distance = 30;
            }
        } else {
            if (robot.jewelColor.isBlue()) {
                robot.drive(-30);
                distance = -30;
            } else {
                robot.drive(30);
                distance = 30;
            }
        }
        robot.jewelArm.setPosition(JEWEL_IN);
        return addedDistance;
    }
    @Override
    public void stopLinearOpMode () {
        robot.jewelArm.setPosition(JEWEL_IN);
        robot.glyphSystem.setPosition(GLYPH_OPENED);
    }

}
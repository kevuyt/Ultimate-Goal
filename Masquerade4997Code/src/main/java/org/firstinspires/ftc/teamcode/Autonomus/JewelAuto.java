package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "Jewel Auto", group = "Group4")
public class JewelAuto extends MasqLinearOpMode implements Constants {
    boolean red;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.initializeAutonomous();
        while (!opModeIsActive()) {
            if (controller1.aOnPress() && !red) {
                 dash.clear();
                 dash.create("THIS IS RED");
                 red = true;
                 controller1.update();
            }
            else if (controller1.aOnPress() && red) {
                 dash.clear();
                 dash.create("THIS IS BLUE");
                 red = false;
                 controller1.update();
            }
            controller1.update();
            dash.update();
        }
        waitForStart();
        MasqExternal.sleep(2000);
        if (!red) {
            robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
            if (robot.jewelColorBlue.isRed()) {
                robot.drive(30);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                robot.turn(30, Direction.RIGHT);
                robot.drive(100);
                robot.drive(-30);
            }
            else {
                robot.drive(-30);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                robot.turn(30, Direction.RIGHT);
                robot.drive(170);
                robot.drive(-30);
            }
        }
        else {
            robot.jewelArmBlue.setPosition(JEWEL_RED_OUT);
            if (robot.jewelColorRed.isBlue()) {
                robot.drive(30);
                robot.jewelArmRed.setPosition(JEWEL_RED_IN);
                robot.turn(30, Direction.LEFT);
                robot.drive(100);
                robot.drive(-30);
            }
            else {
                robot.drive(-30);
                robot.jewelArmBlue.setPosition(JEWEL_RED_IN);
                robot.turn(30, Direction.LEFT);
                robot.drive(170);
                robot.drive(-30);
            }
        }
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        MasqExternal.sleep(10000);
    }
}
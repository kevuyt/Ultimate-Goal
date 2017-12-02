package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 10/5/17.
 */
@Autonomous(name = "Jewel Auto", group = "Autonomus")
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
                //dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        MasqExternal.sleep(2000);
        robot.glyphSystemBottom.setPosition(GLYPH_TOP_CLOSED);
        if (!red) {
            if (robot.jewelColorBlue.isRed()) {
                robot.drive(-30);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);//V2
                robot.turn(90, Direction.LEFT); //V2
                robot.turn(60, Direction.LEFT);
                robot.drive(90);
            } else {
                robot.drive(60);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
            }
        }
        else {
            if (robot.jewelColorBlue.isBlue()) {
                robot.drive(-30);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);//V2
                robot.turn(40, Direction.LEFT); //V2
                robot.drive(90);
            }
            else {
                robot.turn(40, Direction.LEFT);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                robot.drive(90);
            }
        }
        robot.glyphSystemBottom.setPosition(GLYPH_TOP_OPENED);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        MasqExternal.sleep(10000);
    }
}
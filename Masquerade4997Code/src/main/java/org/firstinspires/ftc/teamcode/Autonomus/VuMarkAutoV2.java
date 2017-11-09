package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "VuMarkAutoV2", group = "Autonomus")
public class VuMarkAutoV2 extends MasqLinearOpMode implements Constants {
    private boolean blue;
    public void runLinearOpMode()throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while(!opModeIsActive()) {
            dash.create(robot.vuforia.getVuMark());
            if (controller1.aOnPress() && !blue) {
                dash.clear();
                dash.create("THIS IS RED");
                blue = true;
                controller1.update();
            }
            else if (controller1.aOnPress() && blue) {
                dash.clear();
                dash.create("THIS IS BLUE");
                blue = false;
                controller1.update();
            }
            controller1.update();
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        runJewel();

    }
    public void runJewel () {
        if (blue) {
            if (robot.jewelColor.isRed()) {
                robot.drive(-30);
                robot.jewelArm.setPosition(JEWEL_IN);//V2
                robot.turn(90, Direction.LEFT); //V2
                robot.turn(60, Direction.LEFT);
                robot.drive(90);
            } else {
                robot.drive(60);
                robot.jewelArm.setPosition(JEWEL_IN);
            }
        } else {
            if (robot.jewelColor.isBlue()) {
                robot.drive(-30);
                robot.jewelArm.setPosition(JEWEL_IN);//V2
                robot.turn(40, Direction.LEFT); //V2
                robot.drive(90);
            }
            else {
                robot.turn(40, Direction.LEFT);
                robot.jewelArm.setPosition(JEWEL_IN);
                robot.drive(90);
            }
        }
    }
    public void runVuMark(String vuMark) {
        if (isCenter(vuMark)){

        }

    }
    public final boolean isCenter(String vuMark) {return vuMark.toLowerCase().contains("c");}
    public final boolean isLeft(String vuMark) {return vuMark.toLowerCase().contains("l");}
    public final boolean isRight(String vuMark) {return vuMark.toLowerCase().contains("g");}
 }
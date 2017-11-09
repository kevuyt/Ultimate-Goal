package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/9/17.
 */
@Autonomous(name = "RED VUMARK", group = "Autonomus")
public class RedVuMark extends MasqLinearOpMode implements Constants {
    private boolean blue;
    public void runLinearOpMode()throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        while(!opModeIsActive()) {
            dash.create(INIT_MESSAGE);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        robot.waitForVuMark();
        String vuMark = robot.vuforia.getVuMark();
        runVuMark(vuMark, runJewel());
    }
    public int runJewel () {
        int addedAngle = 0;
        if (blue) {
            if (robot.jewelColor.isRed()) {
                robot.turn(30, Direction.RIGHT);
            } else {
                robot.turn(30, Direction.LEFT);
            }
        } else {
            if (robot.jewelColor.isBlue()) {
                robot.turn(30, Direction.RIGHT);
                addedAngle = -30;
            }
            else {
                robot.turn(30, Direction.LEFT);
                addedAngle = 30;
            }
        }
        return addedAngle;
    }
    public void runVuMark(String vuMark, int addedDistance) {
        if (isCenter(vuMark)){

        }

    }
    public final boolean isCenter(String vuMark) {return vuMark.toLowerCase().contains("c");}
    public final boolean isLeft(String vuMark) {return vuMark.toLowerCase().contains("l");}
    public final boolean isRight(String vuMark) {return vuMark.toLowerCase().contains("g");}
 }
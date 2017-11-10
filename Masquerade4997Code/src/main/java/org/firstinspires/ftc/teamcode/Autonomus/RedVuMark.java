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
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        dash.create(INIT_MESSAGE);
        dash.update();
        waitForStart();
        robot.vuforia.activateVuMark();
        robot.waitForVuMark();
        String vuMark = robot.vuforia.getVuMark();
        dash.create(vuMark);
        dash.update();
        int addedAngle = runJewel();
        runVuMark(vuMark, addedAngle);
    }
    public int runJewel () {
        int addedAngle;
        robot.jewelArm.setPosition(JEWEL_OUT);
        MasqExternal.sleep(2000);
        if (robot.jewelColor.isBlue()) {
            robot.turn(10, Direction.RIGHT);
            addedAngle = 10;
        }
        else {
            robot.turn(10, Direction.LEFT);
            addedAngle = -10;
        }
        robot.jewelArm.setPosition(JEWEL_IN);
        return addedAngle;
    }
    public void runVuMark(String vuMark, int addedDistance) {
        if (MasqExternal.VuMark.isCenter(vuMark)){robot.turn(20 + addedDistance, Direction.LEFT);}
        else if (MasqExternal.VuMark.isLeft(vuMark)){robot.turn(20 + addedDistance, Direction.LEFT);}
        else if (MasqExternal.VuMark.isRight(vuMark)){robot.turn(10 + addedDistance, Direction.LEFT);}
        robot.drive(80, POWER_LOW);
    }

 }
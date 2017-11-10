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
        robot.initializeAutonomus();
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
            robot.turn(20, Direction.LEFT);
            addedAngle = 20;
        }
        else {
            robot.turn(20, Direction.RIGHT);
            addedAngle = -20;
        }
        robot.jewelArm.setPosition(JEWEL_IN);
        return addedAngle;
    }
    public void runVuMark(String vuMark, int addedDistance) {
        robot.turn(90, Direction.LEFT);
        if (MasqExternal.VuMark.isCenter(vuMark)){robot.turn(90 - (30 + addedDistance), Direction.RIGHT);}
        else if (MasqExternal.VuMark.isLeft(vuMark)){robot.turn(90 - (10 + addedDistance), Direction.RIGHT);}
        else if (MasqExternal.VuMark.isRight(vuMark)){robot.turn(90 - (45 + addedDistance), Direction.RIGHT);}
        robot.drive(80, POWER_LOW, Direction.BACKWARD);
    }
}
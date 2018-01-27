package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/27/18.
 */
@Autonomous(name = "BlueAutoF", group = "Autonomus")
public class BlueAutoF extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setPosition(0.7);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        String vuMark = readVuMark();
        runJewel();
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
    }
    public void runJewel() {
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
        robot.sleep(1000);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
        robot.sleep(1000);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1000);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark (String vuMark) {
        if (MasqExternal.VuMark.isRight(vuMark)) robot.turn(30, Direction.RIGHT);
        else if (MasqExternal.VuMark.isCenter(vuMark)) robot.turn(15, Direction.RIGHT);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.turn(5, Direction.RIGHT);
        else robot.turn(15, Direction.RIGHT);
        robot.drive(90);
    }
}
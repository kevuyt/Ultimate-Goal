package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 12/2/17.
 */
@Autonomous(name = "BlueAutoV2", group = "Autonomus")
public class BlueAutoV2 extends MasqLinearOpMode implements Constants {
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
        robot.sleep(1500);
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(COLOR_NOT_SEEN);
        else robot.blueRotator.setPosition(COLOR_SEEN);
        robot.sleep(1500);
        robot.blueRotator.setPosition(ROTATOR_CENTER);
        robot.sleep(1500);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(185, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(165, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(205, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark)) robot.drive(185, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turn(80, Direction.LEFT);
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setPosition(0);
        robot.sleep(500);
        robot.drive(15, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(20, POWER_OPTIMAL, Direction.FORWARD);
    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.Strafe;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;

/**
 * Created by Archish on 3/9/18.
 */
@Autonomous(name = "RedAutoFV2", group = "C")
public class RedAutoFV2 extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setPosition(Flipper.Position.MID);
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
        if (MasqUtils.VuMark.isLeft(vuMark)) leftMulti();
        else if  (MasqUtils.VuMark.isRight(vuMark)) rightMulti();
        else if (MasqUtils.VuMark.isCenter(vuMark)) centerMulti();
        else  centerMulti();
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(1500);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        robot.sleep(1500);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark (String vuMark) {
        robot.drive(20, POWER_LOW, Direction.BACKWARD);
        if (MasqUtils.VuMark.isRight(vuMark)) robot.turn(25, Direction.RIGHT);
        else if (MasqUtils.VuMark.isCenter(vuMark)) robot.strafe(3, Strafe.RIGHT);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.strafe(5, Strafe.RIGHT);
        else robot.strafe(3, Strafe.RIGHT);
        robot.turnAbsolute(30, Direction.LEFT);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
    public void centerMulti () {

    }
    public void leftMulti () {

    }
    public void rightMulti () {

    }
}
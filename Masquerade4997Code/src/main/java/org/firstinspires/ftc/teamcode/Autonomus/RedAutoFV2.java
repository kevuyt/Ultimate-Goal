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
        robot.sleep(250);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(100);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark (String vuMark) {
        robot.drive(20, POWER_LOW, Direction.BACKWARD);
        if (MasqUtils.VuMark.isRight(vuMark)) {
            robot.turn(8, Direction.LEFT);
        }
        else if (MasqUtils.VuMark.isCenter(vuMark)) {
            robot.turnAbsolute(20, Direction.LEFT);
        }
        else if (MasqUtils.VuMark.isLeft(vuMark)) {
            robot.strafe(3, Strafe.RIGHT);
            robot.turnAbsolute(30, Direction.LEFT);
        }
        else {
            robot.strafe(3, Strafe.RIGHT);
            robot.turnAbsolute(30, Direction.LEFT);
        }
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(4, POWER_LOW, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
    public void centerMulti () {
        robot.turnAbsolute(0, Direction.RIGHT);
        robot.drive(3);
        robot.strafe(20, Strafe.RIGHT);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.drive(60, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(40, Direction.RIGHT);
        robot.drive(10, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.flip(1);
    }
    public void leftMulti () {
        robot.turnAbsolute(0, Direction.RIGHT);
        robot.drive(3);
        robot.strafe(23, Strafe.RIGHT);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.turnAbsolute(25, Direction.RIGHT);
        robot.drive(17, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.flip(1);
        robot.drive(5);
    }
    public void rightMulti () {

    }
}
package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/11/18.
 */
@Autonomous(name = "RedAuto", group = "Autonomus")
public class RedAuto extends MasqLinearOpMode implements Constants {
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
    public void runVuMark(String vuMark) {
        robot.stop(robot.ultra, POWER_LOW, Direction.BACKWARD);
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(24, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(6, POWER_OPTIMAL, Direction.FORWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark));
        robot.turn(90, Direction.RIGHT);
        robot.flipper.setPosition(0);
        robot.sleep(1500);
        robot.drive(12, POWER_OPTIMAL, Direction.BACKWARD);
        robot.intake.setPower(INTAKE);
        robot.flipper.setPosition(1);
        robot.drive(70, POWER_HIGH, Direction.FORWARD);
        robot.flipper.setPosition(0.7);
        robot.intake.setPower(0);
        robot.drive(70, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.setPosition(0);
        robot.sleep(500);
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL, Direction.FORWARD);
    }
}
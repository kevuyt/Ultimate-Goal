package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import MasqueradeSubSystems.SubSystems.Flipper;

/**
 * Created by Archish on 2/7/18.
 */
@Autonomous(name = "RedAutoV2", group = "Autonomus")
public class RedAutoV2 extends MasqLinearOpMode implements Constants {
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
        robot.drive(20, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(18, POWER_LOW, Direction.FORWARD);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(16, POWER_OPTIMAL, Direction.BACKWARD, 2);
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(2, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD, 3);
        robot.turn(60 , Direction.RIGHT);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_LOW, Direction.FORWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipper.setPosition(Flipper.Position.IN);
    }
}
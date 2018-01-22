package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqSensors.MasqLimitSwitch;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/19/18.
 */
@Autonomous(name = "BlueAuto_RP", group = "Autonomus")
public class BlueAuto_RP extends MasqLinearOpMode implements Constants {
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
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        robot.sleep(1000);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1000);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        double startAngle = robot.imu.getHeading();
        robot.drive(25, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(20, POWER_LOW, Direction.FORWARD);
        if (MasqExternal.VuMark.isCenter(vuMark)) robot.drive(16, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isLeft(vuMark)) robot.drive(26, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isRight(vuMark)) robot.drive(8, POWER_OPTIMAL, Direction.BACKWARD);
        else if (MasqExternal.VuMark.isUnKnown(vuMark)) robot.drive(14, POWER_OPTIMAL, Direction.BACKWARD);
        double endAngle = robot.imu.getHeading();
        robot.turn(90 + (endAngle - startAngle), Direction.RIGHT);
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setPosition(0.3);
        robot.sleep(1000);
        robot.drive(6, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(3, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipper.setPosition(1);
        /*
        robot.drive(70, POWER_HIGH, Direction.FORWARD);
        robot.flipper.setPosition(0.7);
        robot.intake.setPower(0);
        robot.drive(70, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.setPosition(0);
        robot.sleep(500);
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL, Direction.FORWARD);*/
    }
}

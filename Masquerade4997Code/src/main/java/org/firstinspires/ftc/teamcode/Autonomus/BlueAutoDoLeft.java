package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqExternal.Direction;
import Library4997.MasqExternal.MasqExternal;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/13/18.
 */
@Autonomous(name = "BlueAutoDoLeft", group = "Autonomus")
public class BlueAutoDoLeft extends MasqLinearOpMode implements Constants {
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
        if (robot.jewelColorBlue.isBlue()) robot.blueRotator.setPosition(ROTATOR_BLUE_SEEN);
        else robot.blueRotator.setPosition(ROTATOR_BLUE_NOT_SEEN);
        robot.sleep(1500);
        robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
        robot.sleep(1500);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        robot.drive(165, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turn(90, Direction.LEFT);
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setPosition(0);
        robot.sleep(500);
        robot.drive(15, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(20, POWER_OPTIMAL, Direction.FORWARD);
    }
}
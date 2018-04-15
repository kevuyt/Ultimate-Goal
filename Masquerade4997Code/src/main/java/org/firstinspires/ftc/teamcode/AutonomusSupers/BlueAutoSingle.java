package org.firstinspires.ftc.teamcode.AutonomusSupers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;

/**
 * Created by Archish on 3/8/18.
 */
@Autonomous(name = "BlueAutoSingle", group = "A")
@Disabled
public class BlueAutoSingle extends MasqLinearOpMode implements Constants {
    private double startAngle = 0;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setFlipperPosition(Flipper.Position.MID);
        while (!opModeIsActive()) {
            dash.create("Initialized");
            dash.update();
        }
        waitForStart();
        robot.vuforia.flash(true);
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        String vuMark = readVuMark();
        runJewel();
        robot.vuforia.flash(false);
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(2000);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        robot.sleep(250);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(100);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runVuMark(String vuMark) {
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(27, POWER_OPTIMAL, Direction.FORWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(20, POWER_OPTIMAL, Direction.FORWARD, 2); //FINAL
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(40, POWER_OPTIMAL, Direction.FORWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(40, POWER_OPTIMAL, Direction.FORWARD, 3);
        robot.turnAbsolute(70, Direction.RIGHT);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.intake.setPower(INTAKE);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.drive(50);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(80, Direction.RIGHT);
        robot.flipper.setFlipperPosition(Flipper.Position.MID);
        robot.sleep(1000);
        robot.lift.setDistance(2);
        robot.lift.runToPosition(Direction.BACKWARD, POWER_OPTIMAL);
        robot.drive(15, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip(1);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
    }
}
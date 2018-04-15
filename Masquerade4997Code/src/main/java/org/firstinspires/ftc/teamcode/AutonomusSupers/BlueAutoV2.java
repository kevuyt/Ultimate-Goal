package org.firstinspires.ftc.teamcode.AutonomusSupers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.AutonomusWorlds.Constants;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;

/**
 * Created by Archish on 2/25/18.
 */
@Autonomous(name = "BlueAutoV2", group = "Autonomus")
@Disabled
public class BlueAutoV2 extends MasqLinearOpMode implements Constants {
    private double startAngle = 0;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
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
        if (MasqUtils.VuMark.isLeft(vuMark)) leftMulti();
        else centerMulti();
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(1000);
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
    public void runVuMark(String vuMark) {
        updateStart();
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(23, POWER_OPTIMAL, Direction.BACKWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(8, POWER_OPTIMAL, Direction.BACKWARD, 2); //FINAL
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(32, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(23, POWER_OPTIMAL, Direction.BACKWARD, 3);
        robot.turnRelative(60, Direction.LEFT);
        updateStart();
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(4, POWER_LOW, Direction.FORWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnRelative(30, Direction.LEFT);
        updateStart();
        robot.intake.setPower(INTAKE);
        robot.drive(60);
    }
    public void leftMulti() {
        robot.drive(3);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.drive(10, POWER_HIGH, Direction.BACKWARD);
        robot.turnRelative(30, Direction.LEFT);
        robot.drive(30);
        robot.turnAbsolute(90, Direction.LEFT);
        robot.drive(10, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.MID);
        robot.turnAbsolute(95, Direction.LEFT);
        robot.lift.setDistance(20);
        robot.lift.runToPosition(Direction.BACKWARD, POWER_OPTIMAL);
        robot.drive(45, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip(1);
        robot.drive(5, POWER_LOW);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(10);
    }
    public void centerMulti() {
        robot.drive(3);
        robot.turnAbsolute(90, Direction.LEFT);
        robot.intake.setPower(INTAKE);
        robot.drive(60);
        robot.drive(10, POWER_HIGH, Direction.BACKWARD);
        robot.turnRelative(30, Direction.LEFT);
        robot.drive(30);
        robot.turnAbsolute(90, Direction.LEFT);
        robot.drive(10, POWER_HIGH, Direction.BACKWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.MID);
        robot.turnAbsolute(95, Direction.LEFT);
        robot.lift.setDistance(20);
        robot.lift.runToPosition(Direction.BACKWARD, POWER_OPTIMAL);
        robot.drive(45, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip(1);
        robot.drive(5, POWER_LOW);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(10);
    }
    public void rightMulti () {
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnRelative(8 + (startAngle - endAngle()), Direction.LEFT);
        robot.drive(40, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip();
        robot.drive(5);
        robot.drive(5, POWER_HIGH, Direction.BACKWARD);
        updateStart();
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.drive(60);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnRelative(20 + (endAngle() - startAngle), Direction.RIGHT);
        robot.drive(40, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.flip();
    }
    public double endAngle () {
        return robot.imu.getAbsoluteHeading();
    }
    public void updateStart() {
        startAngle = robot.imu.getAbsoluteHeading();
    }
}

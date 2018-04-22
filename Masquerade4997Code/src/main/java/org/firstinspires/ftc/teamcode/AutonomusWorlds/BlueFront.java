package org.firstinspires.ftc.teamcode.AutonomusWorlds;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;
import SubSystems4997.SubSystems.Gripper;

/**
 * Created by Archish on 3/25/18.
 */
@Autonomous(name = "BlueFront", group = "A")
//TODO Fix right column angle.
public class BlueFront extends MasqLinearOpMode implements Constants {
    double startTicks, endTicks;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.intake.motor1.setStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(OUTAKE);
            }
        });
        robot.intake.motor1.setUnStalledAction(new Runnable() {
            @Override
            public void run() {
                robot.intake.setPower(INTAKE);
            }
        });
        while (!opModeIsActive()) {
            dash.create("Init");
            dash.update();
        }
        waitForStart();
        robot.gripper.setGripperPosition(Gripper.Grip.CLAMP);
        robot.vuforia.flash(true);
        robot.sleep(robot.getDelay());
        robot.vuforia.activateVuMark();
        String vuMark = readVuMark();
        runJewel();
        robot.vuforia.flash(false);
        robot.driveTrain.setClosedLoop(false);
        runVuMark(vuMark);
    }
    public String readVuMark () {
        robot.waitForVuMark();
        return robot.vuforia.getVuMark();
    }
    public void runJewel() {
        robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
        robot.sleep(1000);
        if (robot.jewelColorRed.isRed()) robot.redRotator.setPosition(ROTATOR_RED_NOT_SEEN);
        else robot.redRotator.setPosition(ROTATOR_RED_SEEN);
        robot.sleep(250);
        robot.jewelArmRed.setPosition(JEWEL_RED_IN);
        robot.sleep(100);
    }
    public void runVuMark(String vuMark) {
        int heading = 0;
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        robot.drive(20);
        if (MasqUtils.VuMark.isCenter(vuMark)) {
            robot.go(20, 90, Direction.LEFT, 0, Direction.BACKWARD);
            heading = 130;
        }
        else if (MasqUtils.VuMark.isLeft(vuMark)) {
            heading = 150;
        }
        else if (MasqUtils.VuMark.isRight(vuMark)) {
            robot.go(40, 90, Direction.RIGHT, 0, Direction.BACKWARD);
            heading = 170;
        }
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) {
            robot.go(20, 90, Direction.LEFT, 0, Direction.BACKWARD);
            heading = 130;
        }
        robot.turnAbsolute(heading, Direction.LEFT, 4);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.drive(8, POWER_OPTIMAL, Direction.BACKWARD);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.sleep(250);
                robot.gripper.setGripperPosition(Gripper.Grip.OUT);
            }
        });
        robot.drive(8, POWER_OPTIMAL, Direction.FORWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnRelative(180 - heading, Direction.LEFT);
        robot.imu.reset();
        if (MasqUtils.VuMark.isLeft(vuMark)) robot.go(40, 90, Direction.LEFT, 0, Direction.BACKWARD);
        else if (MasqUtils.VuMark.isCenter(vuMark)) robot.go(20, 90, Direction.LEFT, 0, Direction.BACKWARD);
        robot.turnAbsolute(175, Direction.RIGHT);
        robot.intake.setPower(INTAKE);
        robot.intake.motor1.enableStallDetection();
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, POWER_LOW, Direction.FORWARD);
        startTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        robot.drive(6, POWER_OPTIMAL, Direction.BACKWARD);
        endTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        MasqUtils.sleep(750);
        robot.turnAbsolute(175, Direction.RIGHT);
        robot.gripper.setGripperPosition(Gripper.Grip.CLAMP);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.drive(130, POWER_OPTIMAL, Direction.BACKWARD, 4);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.lift.setDistance(30);
                robot.lift.runToPosition(Direction.BACKWARD, POWER_OPTIMAL);
            }
        });
        robot.gripper.setGripperPosition(Gripper.Grip.OUT);
        robot.sleep(500);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.drive(5);
    }
}
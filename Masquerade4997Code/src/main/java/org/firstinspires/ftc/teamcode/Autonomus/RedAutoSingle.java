package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;

/**
 * Created by Archish on 3/8/18.
 */
@Autonomous(name = "RedAutoSingleV2", group = "A")
public class RedAutoSingle extends MasqLinearOpMode implements Constants {
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.vuforia.initVuforia(hardwareMap);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        robot.initializeAutonomous();
        robot.initializeServos();
        robot.flipper.setPosition(Flipper.Position.MID);
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
        robot.flipper.setPosition(Flipper.Grip.CLAMP);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) robot.drive(14, POWER_OPTIMAL, Direction.BACKWARD, 3);
        else if (MasqUtils.VuMark.isLeft(vuMark)) robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD, 2); //FINAL
        else if (MasqUtils.VuMark.isRight(vuMark)) {robot.drive(8, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) robot.drive(14, POWER_OPTIMAL, Direction.BACKWARD, 3);
        robot.turnAbsolute(60, Direction.RIGHT);
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.flipper.setPosition(Flipper.Grip.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setPosition(Flipper.Position.IN);
        robot.intake.setPower(INTAKE);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -90, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, -90, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(15, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(80, Direction.RIGHT);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.flipper.setPosition(Flipper.Position.MID);
                robot.flipper.setPosition(Flipper.Grip.CLAMP);
                robot.lift.setDistance(20);
                robot.lift.runToPosition(Direction.BACKWARD, POWER_HIGH);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.drive(15, POWER_OPTIMAL, Direction.BACKWARD);
            }
        });
        robot.flipper.setPosition(Flipper.Position.OUT);
        robot.flipper.setPosition(Flipper.Grip.OUT);
        robot.drive(3, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setPosition(Flipper.Position.IN);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.lift.setDistance(20);
                robot.lift.runToPosition(Direction.FORWARD, POWER_HIGH);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.stop(new StopCondition() {
                    @Override
                    public boolean stop() {
                        return robot.singleBlock.stop();
                    }
                }, -90, POWER_OPTIMAL, Direction.FORWARD);
            }
        });
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, -90, POWER_OPTIMAL, Direction.FORWARD);
    }
}
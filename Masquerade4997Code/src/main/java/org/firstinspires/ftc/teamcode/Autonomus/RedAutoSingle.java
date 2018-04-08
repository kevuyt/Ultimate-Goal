package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqUtilities.MasqUtils;
import Library4997.MasqUtilities.StopCondition;
import Library4997.MasqWrappers.MasqLinearOpMode;
import SubSystems4997.SubSystems.Flipper;
import SubSystems4997.SubSystems.Gripper.Grip;

/**
 * Created by Archish on 3/8/18.
 */
@Autonomous(name = "RedAutoSingleV2", group = "A")
public class RedAutoSingle extends MasqLinearOpMode implements Constants {
    double startTicks = 0, endTicks = 0;
    boolean secondCollection = false;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.lineDetector.setMargin(20);
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
        robot.flipper.setFlipperPosition(Flipper.Position.MID);
        while (!opModeIsActive()) {
            dash.create("Stalled: ");
            dash.update();
        }
        waitForStart();
        robot.intake.setPower(INTAKE);
        robot.intake.motor1.enableStallDetection();
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
        robot.intake.setPower(INTAKE);
        robot.gripper.setGripperPosition(Grip.CLAMP);
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) {
            robot.drive(22, POWER_OPTIMAL, Direction.BACKWARD, 3);
            scoreRightFirstDump();
            scoreLeftSecondDump();
        }
        else if (MasqUtils.VuMark.isLeft(vuMark)) {
            robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD, 2); //FINAL
            scoreRightFirstDump();
        }
        else if (MasqUtils.VuMark.isRight(vuMark)) {
            robot.drive(8, POWER_OPTIMAL, Direction.BACKWARD, 2);}
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) {
            robot.drive(22, POWER_OPTIMAL, Direction.BACKWARD, 3);
            scoreRightFirstDump();
            scoreLeftSecondDump();
        }
    }
    public void scoreRightFirstDump() {
        robot.turnAbsolute(60, Direction.RIGHT);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnAbsolute(90, Direction.RIGHT);
        startTicks = robot.driveTrain.getCurrentPosition();
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        if (robot.doubleBlock.stop()) {
            secondCollection = true;
            robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
            robot.turnAbsolute(70, Direction.RIGHT);
            robot.go(new StopCondition() {
                @Override
                public boolean stop() {
                    return robot.doubleBlock.stop();
                }
            }, 45, Direction.LEFT, 0, Direction.FORWARD);
        }
        int heading = 85;
        robot.turnAbsolute(heading, Direction.RIGHT);
        endTicks = robot.driveTrain.getCurrentPosition();
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                if (!secondCollection) robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
                else robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.gripper.setGripperPosition(Grip.CLAMP);
                robot.flipper.setFlipperPosition(Flipper.Position.OUT);
                robot.sleep(2000);
                robot.gripper.setGripperPosition(Grip.OUT);
            }
        });
        robot.flipper.setFlipperPosition(Flipper.Position.IN);

    }
    public void scoreLeftFirstDump() {
        robot.turnAbsolute(60, Direction.RIGHT);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnAbsolute(90, Direction.RIGHT);
        startTicks = robot.driveTrain.getCurrentPosition();
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        if (robot.doubleBlock.stop()) {
            secondCollection = true;
            robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
            robot.turnAbsolute(70, Direction.RIGHT);
            robot.stop(new StopCondition() {
                @Override
                public boolean stop() {
                    return robot.doubleBlock.stop();
                }
            }, -70, POWER_LOW, Direction.FORWARD);
        }
        int heading = 110;
        robot.turnAbsolute(heading, Direction.RIGHT);
        endTicks = robot.driveTrain.getCurrentPosition();
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                if (!secondCollection) robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
                else robot.drive(48, POWER_OPTIMAL, Direction.BACKWARD);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.gripper.setGripperPosition(Grip.CLAMP);
                robot.flipper.setFlipperPosition(Flipper.Position.OUT);
                robot.sleep(2000);
                robot.gripper.setGripperPosition(Grip.OUT);
            }
        });
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        /*robot.turnAbsolute(90, Direction.RIGHT);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(3, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);*/
    }
    public void scoreLeftSecondDump() {
        int heading = 85;
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -heading, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(120, Direction.RIGHT);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.doubleBlock.stop();
            }
        }, -120, POWER_LOW, Direction.FORWARD);
        /*robot.turnAbsolute(90, Direction.RIGHT);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(3, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5);*/
    }
}
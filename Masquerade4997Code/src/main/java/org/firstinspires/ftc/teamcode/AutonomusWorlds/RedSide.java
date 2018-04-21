package org.firstinspires.ftc.teamcode.AutonomusWorlds;

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
@Autonomous(name = "RedSide", group = "B")
public class RedSide extends MasqLinearOpMode implements Constants {
    double startTicks = 0, endTicks = 0;
    boolean secondCollection = false;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.redLineDetector.setMargin(20);
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
            dash.create("Line Block: ", robot.doubleBlock.stop());
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
        robot.redRotator.setPosition(ROTATOR_RED_CENTER);
        if (MasqUtils.VuMark.isCenter(vuMark)) center();
        else if (MasqUtils.VuMark.isLeft(vuMark)) left();
        else if (MasqUtils.VuMark.isRight(vuMark)) right();
        else if (MasqUtils.VuMark.isUnKnown(vuMark)) center();
    }
    public void centerMulti() {
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.intake.motor1.enableStallDetection();
        robot.intake.setPower(INTAKE);
        robot.yWheel.resetEncoder();
        robot.setYTarget(robot.yWheel.getPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        endCollection();
        robot.turnAbsolute(80, Direction.RIGHT);
        startCollection();
        robot.drive(50, POWER_OPTIMAL, Direction.BACKWARD, 2);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(5, POWER_OPTIMAL);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnAbsolute(60, Direction.RIGHT);
        startTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -60, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(110, Direction.RIGHT);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -110, POWER_LOW, Direction.FORWARD);
        endCollection();
        endTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        MasqUtils.sleep(750);
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.strafeToY(POWER_LOW);
        robot.turnAbsolute(80, Direction.RIGHT);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.drive(50, POWER_OPTIMAL, Direction.BACKWARD, 3);
            }
        }, new Runnable() {
            @Override
            public void run() {
                startCollection();
                robot.sleep(500);
                robot.gripper.setGripperPosition(Grip.CLAMP);
                robot.flipper.setFlipperPosition(Flipper.Position.OUT);
            }
        });
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.turnRelative(15, Direction.LEFT);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(5, POWER_OPTIMAL);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL);
        robot.intake.setPower(INTAKE);
    }
    public void leftMulti() {
        robot.turnAbsolute(80, Direction.RIGHT);
        startCollection();
        robot.yWheel.resetEncoder();
        robot.setYTarget(robot.yWheel.getPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -80, POWER_LOW, Direction.FORWARD);
        endCollection();
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.strafeToY(POWER_OPTIMAL);
        robot.turnAbsolute(70, Direction.RIGHT);
        startCollection();
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.drive(50, POWER_OPTIMAL, Direction.BACKWARD, 3);
                robot.turnRelative(20, Direction.LEFT);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.sleep(500);
                robot.gripper.setGripperPosition(Grip.CLAMP);
                robot.flipper.setFlipperPosition(Flipper.Position.OUT);
            }
        });
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(5, POWER_OPTIMAL);
        robot.drive(4, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.turnAbsolute(70, Direction.RIGHT);
        startTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -70, POWER_LOW, Direction.FORWARD);
        robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(110, Direction.RIGHT);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -110, POWER_LOW, Direction.FORWARD, 1.5);
        endTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        endCollection();
        robot.turnAbsolute(90, Direction.RIGHT);
        robot.strafeToY(POWER_LOW);
        startCollection();
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                if (secondCollection) {
                    robot.strafeToY(POWER_OPTIMAL);
                    robot.drive(50, POWER_HIGH, Direction.BACKWARD, 3);
                }
                robot.drive(45, POWER_HIGH, Direction.BACKWARD, 3);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.gripper.setGripperPosition(Grip.CLAMP);
                robot.flipper.setFlipperPosition(Flipper.Position.OUT);
            }
        });
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                MasqUtils.sleep(250);
                robot.drive(10, POWER_OPTIMAL, Direction.BACKWARD);
                robot.drive(10, POWER_LOW, Direction.FORWARD);
                robot.flipper.setFlipperPosition(Flipper.Position.IN);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.sleep(1000);
                robot.gripper.setGripperPosition(Grip.OUT);
                robot.sleep(500);
            }
        });
        robot.intake.setPower(INTAKE);
    }
    public void rightMulti() {
        robot.turnAbsolute(90, Direction.RIGHT);
        startCollection();
        robot.yWheel.resetEncoder();
        robot.setYTarget(robot.yWheel.getPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -90, POWER_LOW, Direction.FORWARD);
        endCollection();
        robot.turnAbsolute(90, Direction.RIGHT);
        super.runSimultaneously(new Runnable() {
            @Override
            public void run() {
                robot.drive(50, POWER_OPTIMAL, Direction.BACKWARD, 2);
            }
        }, new Runnable() {
            @Override
            public void run() {
                robot.sleep(500);
                startCollection();
            }
        });
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.turnRelative(10, Direction.LEFT);
        robot.drive(4, POWER_OPTIMAL);
        robot.drive(5, POWER_OPTIMAL, Direction.BACKWARD);
        robot.drive(6, POWER_OPTIMAL);
        robot.flipper.setFlipperPosition(Flipper.Position.IN);
        robot.intake.motor1.setStallDetection(true);
        robot.intake.setPower(INTAKE);
        robot.turnAbsolute(70, Direction.RIGHT);
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return robot.singleBlock.stop();
            }
        }, -70, POWER_LOW, Direction.FORWARD);
        robot.drive(20, POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(70, Direction.RIGHT);
        startTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        robot.stop(new StopCondition() {
            @Override
            public boolean stop() {
                return secondBlock();
            }
        }, -70, POWER_LOW, Direction.FORWARD);
        endCollection();
        endTicks = Math.abs(robot.driveTrain.getCurrentPosition());
        MasqUtils.sleep(750);
        robot.sleep(100);
        if (!secondCollection) robot.drive(((endTicks - startTicks) / MasqUtils.CLICKS_PER_INCH), POWER_OPTIMAL, Direction.BACKWARD);
        robot.turnAbsolute(75, Direction.RIGHT);
        startCollection();
        robot.gripper.setGripperPosition(Grip.CLAMP);
        robot.flipper.setFlipperPosition(Flipper.Position.OUT);
        robot.strafeToY(POWER_LOW);
        if (secondCollection) robot.drive(50, .8, Direction.BACKWARD, 3);
        robot.drive(45, .8, Direction.BACKWARD, 3);
        robot.gripper.setGripperPosition(Grip.OUT);
        robot.drive(4, POWER_OPTIMAL, Direction.FORWARD);
        robot.drive(7, POWER_LOW, Direction.BACKWARD);
        robot.drive(5, POWER_OPTIMAL, Direction.FORWARD);
        robot.intake.setPower(INTAKE);
    }
    boolean secondBlock () {
        return robot.doubleBlock.stop();
    }
    void startCollection () {
        robot.intake.motor1.setStallDetection(true);
        robot.intake.setPower(INTAKE);
    }
    void endCollection () {
        int i = 0;
        while (i < 20) {
            robot.intake.motor1.setStallDetection(false);
            robot.intake.setPower(0);
            i++;
        }
    }
    public void center () {
        robot.drive(34, POWER_OPTIMAL, Direction.BACKWARD, 3);
        centerMulti();
    }
    public void right () {
        robot.drive(30, POWER_OPTIMAL, Direction.BACKWARD, 2);
        rightMulti();
    }
    public void left () {
        robot.drive(44, POWER_OPTIMAL, Direction.BACKWARD, 3); //FINAL
        leftMulti();
    }
}
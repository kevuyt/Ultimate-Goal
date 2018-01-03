package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

@TeleOp(name = "NFSV1", group = "Autonomus")
public class NFSV1 extends MasqLinearOpMode implements Constants {
    boolean jewelArmInRed = true, jewelArmInBlue = true, clawClosed = true;
    double currentFlipPosition = 0, flipIncrement = 0.02;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        robot.intake.setPower(INTAKE);
        while (opModeIsActive()) {
            robot.MECH(controller1);
            if (controller1.rightBumper()) robot.intake.setPower(INTAKE);
            else if (controller1.leftBumper()) robot.intake.setPower(OUTAKE);
            if (controller2.leftBumper()) robot.relicAdjuster.setPower(0.5);
            else if (controller2.leftTriggerPressed()) robot.relicAdjuster.setPower(-0.5);
            else robot.relicAdjuster.setPower(0);
            if (controller2.xOnPress() && jewelArmInRed) {
                jewelArmInRed = false;
                robot.jewelArmRed.setPosition(JEWEL_RED_OUT);
                controller2.update();
            }
            else if (controller2.xOnPress() && !jewelArmInRed) {
                jewelArmInRed = true;
                robot.jewelArmRed.setPosition(JEWEL_RED_IN);
                controller2.update();
            }
            if (controller2.bOnPress() && jewelArmInBlue) {
                jewelArmInBlue = false;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_OUT);
                controller2.update();
            }
            else if (controller2.bOnPress() && !jewelArmInBlue) {
                jewelArmInBlue = true;
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
                controller2.update();
            }
            if (controller2.aOnPress() && clawClosed) {
                clawClosed = false;
                robot.relicGripper.setPosition(CLAW_OPENED);
                controller2.update();
            }
            else if (controller2.aOnPress() && !clawClosed) {
                clawClosed = true;
                robot.relicGripper.setPosition(CLAW_CLOSED);
                controller2.update();
            }
            if (controller2.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.rightBumper()) robot.relicLift.setPower(LIFT_UP/2);
            else if (controller2.rightTriggerPressed()) {robot.relicLift.setPower(LIFT_DOWN);}
            else robot.relicLift.setPower(0);
            if (controller2.rightStickY() < 0) currentFlipPosition += flipIncrement;
            if (controller2.rightStickY() > 0) currentFlipPosition -= flipIncrement;
            robot.flipper.setPosition(currentFlipPosition);
            controller1.update();
            controller2.update();
            robot.sleep(50);
        }
    }
}
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqExternal.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 1/26/18.
 */
@TeleOp(name = "MECHV2", group = "Autonomus")
public class MECHV2 extends MasqLinearOpMode implements Constants {
    boolean clawClosed = true, fliperFineMode;
    double currentFlipPosition = 0, flipIncrement = 0.05, currentRelicPower = LIFT_UP;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.initializeTeleop();
        robot.relicLift.setClosedLoop(false);
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            if (controller1.leftBumper()) robot.MECH(controller1, Direction.BACKWARD);
            else robot.MECH(controller1, Direction.FORWARD);
            if (controller2.a()) fliperFineMode = true;
            else fliperFineMode = false;
            if (controller1.rightBumper()) robot.intake.setPower(INTAKE);
            else if (controller1.rightTriggerPressed()) robot.intake.setPower(OUTAKE);
            else robot.intake.setPower(0);
            if (controller2.leftStickY() < 0) robot.relicAdjuster.setPower(0.25);
            else if (controller2.leftStickY() > 0) robot.relicAdjuster.setPower(-0.25);
            else robot.relicAdjuster.setPower(0);
            if (controller1.x()) {
                robot.jewelArmRed.setPosition(JEWEL_RED_IN);
                robot.jewelArmBlue.setPosition(JEWEL_BLUE_IN);
            }
            if (controller2.bOnPress() && clawClosed) {
                clawClosed = false;
                robot.relicGripper.setPosition(CLAW_OPENED);
                controller2.update();
            }
            else if (controller2.bOnPress() && !clawClosed) {
                clawClosed = true;
                robot.relicGripper.setPosition(CLAW_CLOSED);
                controller2.update();
            }
            if (controller2.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.x()) currentRelicPower = LIFT_UP / 3;
            else currentRelicPower = LIFT_UP;
            if (controller2.leftBumper()) robot.relicLift.setPower(currentRelicPower);
            else if (controller2.leftTriggerPressed()) {robot.relicLift.setPower(-currentRelicPower);}
            else robot.relicLift.setPower(0);
            if (fliperFineMode) {
                if (controller2.rightStickY() < 0) currentFlipPosition += flipIncrement;
                if (controller2.rightStickY() > 0) currentFlipPosition -= flipIncrement;
                if (currentFlipPosition > 1) currentFlipPosition = 1;
                if (currentFlipPosition < 0) currentFlipPosition = 0;
                robot.flipper.setPosition(currentFlipPosition);
            }
            else {
                if (controller2.rightStickY() < -.5) robot.flipper.setPosition(0);
                else if (controller2.rightStickY() > .5) robot.flipper.setPosition(1);
                else if (controller2.rightStickX() > .5) robot.flipper.setPosition(.5);
                else if (controller2.rightStickX() < -.5) robot.flipper.setPosition(.7);
            }
            controller1.update();
            controller2.update();
            robot.sleep(50);
        }
    }
}
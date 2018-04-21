package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECHV4", group = "Autonomus")
public class MECHV4 extends MasqLinearOpMode implements Constants {
    double currentRelicPower = LIFT_UP, position = 1;
    Direction direction = Direction.FORWARD;
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        robot.blueLineDetector.setPassive();
        robot.redLineDetector.setPassive();
        robot.jewelColorRed.setPassive();
        robot.singleBlock.setPassive();
        robot.doubleBlock.setPassive();
        robot.initializeTeleop();
        robot.relicLift.setClosedLoop(false);
        while (!opModeIsActive()) {
            dash.create("READY TO RUN: ", robot.jewelColorRed.getBlue());
            dash.create("Front Left: ", robot.driveTrain.leftDrive.motor1.getCurrentPosition());
            dash.create("Front Right: ", robot.driveTrain.rightDrive.motor1.getCurrentPosition());
            dash.create("Back Left: ", robot.driveTrain.leftDrive.motor2.getCurrentPosition());
            dash.create("Back Right: ", robot.driveTrain.rightDrive.motor2.getCurrentPosition());
            dash.update();
        }
        waitForStart();
        robot.relicAdjuster.setPosition(0);
        while (opModeIsActive()) {
            if (controller1.y()) direction = Direction.FORWARD;
            if (controller1.b()) direction = Direction.BACKWARD;
            if (controller1.rightBumper()) robot.intake.setPower(INTAKE);
            else if (controller1.rightTriggerPressed()) robot.intake.setPower(OUTAKE);
            else  robot.intake.setPower(0);
            if (controller2.leftStickY() < -0.5) position = 0;
            else if (controller2.leftStickY() > 0.5) position = .98;
            else if (controller2.leftStickX() > 0.5) position = 0.5;
            else if (controller2.leftStickX() < -0.5) position = 0.5;
            robot.relicAdjuster.setPosition(position);
            if (controller1.x()) robot.jewelArmRed.setPosition(JEWEL_RED_IN);
            if (controller2.b()) robot.relicGripper.setPosition(CLAW_OPENED);
            else if (controller2.y()) robot.relicGripper.setPosition(CLAW_CLOSED);
            if (controller2.rightBumper()) robot.lift.setPower(LIFT_UP);
            else if (controller2.rightTriggerPressed()) robot.lift.setPower(LIFT_DOWN);
            else robot.lift.setPower(0);
            if (controller2.leftBumper()) robot.relicLift.setPower(currentRelicPower);
            else if (controller2.leftTriggerPressed()) {robot.relicLift.setPower(-controller2.leftTrigger());}
            else robot.relicLift.setPower(0);
            robot.flipper.DriverControl(controller2);
            robot.MECH(controller1, direction, false);
        }
    }
}
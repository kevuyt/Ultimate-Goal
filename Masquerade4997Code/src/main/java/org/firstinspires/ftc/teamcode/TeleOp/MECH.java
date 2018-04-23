package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.MasqUtilities.Direction;
import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 2/7/18.
 */
@TeleOp(name = "MECH", group = "Autonomus")
public class MECH extends MasqLinearOpMode implements Constants {
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
        while (opModeIsActive()) {
            robot.MECH(controller1);
        }
    }
}
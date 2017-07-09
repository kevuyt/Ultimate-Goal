package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Library4997.RobotTwo;


/**
 * This is a basic template copy and paste this class for any TeleOp,
 * refactor the file name to match the TeleOp class title
 */

@TeleOp(name = "DefenseTeleop", group = "Template")
@Disabled
public class DefenseAuto extends LinearOpMode implements Constants{
    private RobotTwo robot = new RobotTwo();
    public void runOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            telemetry.addLine("Hello");
            telemetry.update();
        }
        waitForStart();
        
    }
}

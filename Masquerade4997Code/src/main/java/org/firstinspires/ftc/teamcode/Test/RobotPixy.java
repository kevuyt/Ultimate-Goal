package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robots.Falcon.Falcon;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Michael Vierra, FTC 8461 on 9/13/2017.
 */

/*
Bytes    16-bit word    Description
        ----------------------------------------------------------------
        0, 1     y              sync: 0xaa55=normal object, 0xaa56=color code object
        2, 3     y              checksum (sum of all 16-bit words 2-6, that is, bytes 4-13)
        4, 5     y              signature number
        6, 7     y              x center of object
        8, 9     y              y center of object
        10, 11   y              width of object
        12, 13   y              height of object
*/
@Autonomous(name = "PIXY1", group = "test")
public class RobotPixy extends MasqLinearOpMode {
    Falcon robot = new Falcon();
    public void runLinearOpMode() throws InterruptedException {
        robot.mapHardware(hardwareMap);
        while (!opModeIsActive()) {
            telemetry.addLine("HOLA");
            telemetry.update();
        }
        waitForStart();
        int x= 0;
        while(opModeIsActive()) {
            robot.NFS(controller1);
            //for (byte b : robot.pixy.getOut()) {
                //dash.create("Byte: " + Integer.toString(x), b);
                //++;
            //}
            x = 0;
            dash.create("Current X: ", robot.tracker.getGlobalX());
            dash.create("Current Y: ", robot.tracker.getGlobalY());
            robot.update();
        }
    }
}
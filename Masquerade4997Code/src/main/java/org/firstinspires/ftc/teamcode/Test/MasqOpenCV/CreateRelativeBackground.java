package org.firstinspires.ftc.teamcode.Test.MasqOpenCV;

import android.app.Activity;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomus.Constants;

import Library4997.MasqWrappers.MasqLinearOpMode;

/**
 * Created by Archish on 11/11/17.
 */
@TeleOp(name = "CreateRelativeBackground", group = "Autonomus")
public class CreateRelativeBackground extends MasqLinearOpMode implements Constants {
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
    public void runLinearOpMode() throws InterruptedException {
        while (!opModeIsActive()) {
            dash.create(robot.imu);
            dash.update();
        }
        waitForStart();
        while (opModeIsActive()) {
            robot.openCV.getView(relativeLayout);
        }
    }
}
package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import Library4997.MasqSensors.MasqVuforia;
import Library4997.MasqWrappers.MasqLinearOpMode;


@TeleOp
public class WebcamTeleop extends MasqLinearOpMode {

    @Override
    public void runLinearOpMode() throws InterruptedException {
        MasqVuforia vuforia = new MasqVuforia(true);
        TFObjectDetector tfod = vuforia.tfod( "Quad", "Single");
        tfod.setClippingMargins(160,230,400,300);
        tfod.activate();

        while(!opModeIsActive()) {
            float height = 0;
            if(tfod.getRecognitions().size() > 0) {
                for (Recognition recognition : tfod.getRecognitions()) {
                    if (recognition.getHeight() > height) height = recognition.getHeight();
                }
            }
            dash.create("Height: " + height);
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            float height = 0;
            if(tfod.getRecognitions().size() > 0) {
                for (Recognition recognition : tfod.getRecognitions()) {
                    if (recognition.getHeight() > height) height = recognition.getHeight();
                }
            }
            dash.create("Height: " + height);
            dash.update();
        }
        tfod.shutdown();
    }
}
package org.firstinspires.ftc.teamcode.Robots.MarkOne.Test;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Robots.MarkOne.Robot.MarkOne;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import Library4997.MasqWrappers.MasqLinearOpMode;

import static Library4997.MasqResources.MasqUtils.VUFORIA_KEY;

/**
 * Created by Keval Kataria on 11/1/2019
 */

@TeleOp(name = "CameraTestTeleop", group = "Prototype")
public class CameraTestTeleop extends MasqLinearOpMode {
    private MarkOne robot = new MarkOne();
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;
    @Override
    public void runLinearOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.detector.start();

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        targetsSkyStone.activate();
        while(!opModeIsActive()) {
            dash.create("Hello");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
           // robot.detector.skystoneDetector.setInput(readFrame());
        }
    }

    public Mat readFrame() {
        VuforiaLocalizer.CloseableFrame frame;
        Image rgb = null;
        try {
            frame = vuforia.getFrameQueue().take();
        } catch (InterruptedException e) {
            e.printStackTrace();
            return null;
        }
        long numImages = frame.getNumImages();
        for(int i = 0; i < numImages; i++) {
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, mat);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        frame.close();
        return mat;
    }
}

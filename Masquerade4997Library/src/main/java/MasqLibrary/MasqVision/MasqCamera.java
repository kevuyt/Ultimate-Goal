package MasqueradeLibrary.MasqVision;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import static java.util.Locale.US;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Keval Kataria on 6/1/2020
 */

public class MasqCamera {
    private OpenCvWebcam camera;
    public MasqCVDetector detector;
    private boolean streaming = false, paused = false;

    public MasqCamera(MasqCVDetector detector, HardwareMap hardwareMap) {
        this.detector = detector;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(detector);
    }

    public void start(OpenCvCameraRotation rotation) {
        camera.openCameraDevice();
        camera.startStreaming(1280, 960, rotation);
        streaming = true;
    }

    public void start() {start(UPRIGHT);}

    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
        streaming = false;
    }

    public void pause() {
        camera.pauseViewport();
        paused = true;
    }

    public void resume() {
        camera.resumeViewport();
        paused = false;
    }

    public OpenCvWebcam getCamera() {return camera;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "Webcam:\nStreaming: %s\nPaused: %s", streaming ? "Yes" :"No", paused ? "Yes" : "No");
    }
}
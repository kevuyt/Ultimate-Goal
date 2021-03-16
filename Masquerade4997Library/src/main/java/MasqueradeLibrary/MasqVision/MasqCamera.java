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
    private OpenCvWebcam webCam;
    public MasqCVDetector detector;
    private boolean streaming = false, paused = false;

    public MasqCamera(MasqCVDetector detector, HardwareMap hardwareMap) {
        this.detector = detector;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.setPipeline(detector);
    }

    public void start(OpenCvCameraRotation rotation) {
        webCam.openCameraDevice();
        webCam.startStreaming(1280, 960, rotation);
        streaming = true;
    }

    public void start() {start(UPRIGHT);}

    public void stop() {
        webCam.stopStreaming();
        webCam.closeCameraDevice();
        streaming = false;
    }

    public void pause() {
        webCam.pauseViewport();
        paused = true;
    }

    public void resume() {
        webCam.resumeViewport();
        paused = false;
    }

    public OpenCvWebcam getCamera() {return webCam;}

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "Webcam:\nStreaming: %s\nPaused: %s", streaming ? "Yes" :"No", paused ? "Yes" : "No");
    }
}
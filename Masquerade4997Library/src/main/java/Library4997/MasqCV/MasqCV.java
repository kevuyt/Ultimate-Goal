package Library4997.MasqCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import Library4997.MasqCV.detectors.MasqCVDetector;


/**
 * Contains global values and types
 */
public class MasqCV {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webcam;
    public MasqCVDetector detector;
    private Cam cam;

    public enum Cam{
        PHONE, WEBCAM
    }

    public MasqCV(MasqCVDetector detector, Cam cam, HardwareMap hwMap){
        this.cam = cam;
        this.detector = detector;
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        if(cam.equals(Cam.PHONE)){
            phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            phoneCamera.setPipeline(detector);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam = new OpenCvWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(detector);
        }
    }

    public void start(OpenCvCameraRotation rotation){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320,240, rotation);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.openCameraDevice();
            webcam.startStreaming(320,240, rotation);
        }
    }
    public void start() {
        start(OpenCvCameraRotation.UPRIGHT);
    }

    public void stop(){
        if(cam.equals(Cam.PHONE)){
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pauseViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.pauseViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.pauseViewport();
    }

    public void resumeViewPort(){
        if(cam.equals(Cam.PHONE)) phoneCamera.resumeViewport();
        else if(cam.equals(Cam.WEBCAM)) webcam.resumeViewport();
    }
}

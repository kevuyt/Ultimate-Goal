package Library4997.MasqCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import Library4997.MasqCV.detectors.MasqCVDetector;


/**
 * Created by Keval Kataria on 6/1/2020
 */
public class MasqCamera {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webcam;
    public MasqCVDetector detector;
    private Cam cam;

    public enum Cam{
        PHONE, WEBCAM
    }

    public MasqCamera(MasqCVDetector detector, Cam cam, HardwareMap hardwareMap){
        this.cam = cam;
        this.detector = detector;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if(cam.equals(Cam.PHONE)){
            phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCamera.setPipeline(detector);
        }
        else if(cam.equals(Cam.WEBCAM)){
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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
            webcam.startStreaming(1280,960, rotation);
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

package Library4997.MasqVision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import static Library4997.MasqVision.MasqCamera.Cam.*;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Keval Kataria on 6/1/2020
 */
public class MasqCamera {
    private OpenCvCamera phoneCam;
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
        if(cam.equals(PHONE)){
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCam.setPipeline(detector);
        }
        else if(cam.equals(WEBCAM)){
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(detector);
        }
    }

    public void start(OpenCvCameraRotation rotation){
        if(cam.equals(PHONE)){
            phoneCam.openCameraDevice();
            phoneCam.startStreaming(320,240, rotation);
        }
        else if(cam.equals(WEBCAM)){
            webcam.openCameraDevice();
            webcam.startStreaming(1280, 960, rotation);
        }
    }
    public void start() {
        start(UPRIGHT);
    }

    public void stop(){
        if(cam.equals(PHONE)){
            phoneCam.stopStreaming();
            phoneCam.closeCameraDevice();
        }
        else if(cam.equals(WEBCAM)){
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pause(){
        if(cam.equals(PHONE)) phoneCam.pauseViewport();
        else if(cam.equals(WEBCAM)) webcam.pauseViewport();
    }

    public void resume(){
        if(cam.equals(PHONE)) phoneCam.resumeViewport();
        else if(cam.equals(WEBCAM)) webcam.resumeViewport();
    }

}

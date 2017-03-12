package BasicLib4997.MasqSensors;

import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import BasicLib4997.MasqHardware;
import BasicLib4997.MasqMotors.MasqRobot.MasqRobot;

/**
 * Created by Archish on 3/12/17.
 */

public class MasqVuforia implements MasqHardware, Sensor_Thresholds{
    String name, target1, target2, target3, asset;
    VuforiaLocalizer vuforia;
    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;
    OpenGLMatrix lastLocation = null;

    public MasqVuforia (String name, String target1, String target2, String target3, String asset){
        this.name = name;
        this.target1 = target1;
        this.target2 = target2;
        this.target3 = target3;
        this.asset = asset;
        setUp();
    }

    private void setUp () {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables target = this.vuforia.loadTrackablesFromAsset(asset);
        VuforiaTrackable targetOne = target.get(0);
        targetOne.setName(target1);
        VuforiaTrackable targetTwo  = target.get(1);
        targetTwo.setName(target2);
        VuforiaTrackable targetThree = target.get(2);
        targetThree.setName(target3);
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(target);

    }





























    public String getName() {
        return name;
    }

    public String[] getDash() {
        return new String[0];
    }
}

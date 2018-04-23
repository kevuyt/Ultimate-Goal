package Library4997.MasqSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import Library4997.MasqUtilities.MasqHardware;
import Library4997.MasqUtilities.MasqUtils;

/**
 * Created by Archish on 11/7/17.
 */

public class MasqVuforiaBeta implements MasqHardware {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;
    private VuforiaTrackable relicTemplate;
    private HardwareMap hardwareMap;
    public void initVuforia(HardwareMap pHardwareMap) {
        hardwareMap = pHardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = MasqUtils.VUFORIA_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        trackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = trackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
    }
    public void activateVuMark() {
        trackables.activate();
    }
    public String getVuMark() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return String.format("%s", vuMark);
    }

    public void flash(boolean on) {
        CameraDevice.getInstance().setFlashTorchMode(on);
    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}

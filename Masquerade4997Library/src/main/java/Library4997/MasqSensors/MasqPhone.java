package Library4997.MasqSensors;

import android.hardware.Camera;

/**
 * Created by Archish on 2/14/18.
 */

public class MasqPhone {
    Camera cam = Camera.open();
    Camera.Parameters p = cam.getParameters();
    public MasqPhone () {
        p.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
    }
    public void flashLightOn () {
        cam.setParameters(p);
        cam.startPreview();
    }
    public void flashLightOff() {
        cam.stopPreview();
        cam.release();
    }
}

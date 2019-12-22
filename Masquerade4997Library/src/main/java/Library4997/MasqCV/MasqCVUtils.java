package Library4997.MasqCV;

import android.graphics.Bitmap;

import com.vuforia.Image;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by Archishmaan Peyyety on 2019-11-25.
 * Project: MasqLib
 */
public class MasqCVUtils {
    public static Mat cvtBitmapMat(Bitmap bm, Image rgb) {
        bm.copyPixelsFromBuffer(rgb.getPixels());
        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, mat);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);
        return mat;
    }
    public enum DetectionSpeed {
        VERY_FAST,
        FAST,
        BALANCED,
        SLOW,
        VERY_SLOW
    }

    public enum AreaScoringMethod {
        MAX_AREA,
        PERFECT_AREA,
        COLOR_DEVIATION
    }

    public enum CameraMode {
        BACK,
        FRONT,
        WEBCAM
    }

    public enum VuMark {
        NONE,
        BLUE_ROVER,
        RED_FOOTPRINT,
        FRONT_CRATERS,
        BACK_SPACE
    }
}

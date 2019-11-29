package MasqCV;

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
}

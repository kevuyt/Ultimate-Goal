package Library4997.MasqSensors;

import com.vuforia.Vec3F;

import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import static org.opencv.imgcodecs.Imgcodecs.CV_LOAD_IMAGE_COLOR;

/**
 * Created by Archish on 9/7/17.
 */

public class MasqOpenCV {
    String targetOne, targetTwo, targetThree, asset, assetPath;
    Mat img;
    Vec3F pixel;
    public MasqOpenCV(String t1, String t2, String t3, String asset){
        targetOne = t1;
        targetTwo = t2;
        targetThree = t3;
        this.asset = asset;
        img = Imgcodecs.imread(getPathName(), CV_LOAD_IMAGE_COLOR);
    }
    public MasqOpenCV(String t1, String t2, String asset){
        targetOne = t1;
        targetTwo = t2;
        targetThree = null;
        this.asset = asset;
    }
    public MasqOpenCV(String t1, String asset){
        targetOne = t1;
        targetTwo = null;
        targetThree = null;
        this.asset = asset;
    }
    public int[] rbg (){
        int[] rbgReturn = new int[]{0};
        return rbgReturn;
    }
    public boolean isSeen(String target){return true;}
    public String getTargetSeen(){
        if (isSeen(targetOne)){
            return targetOne;
        } else if (isSeen(targetTwo) && targetTwo != null){
            return targetTwo;
        } else if (isSeen(targetThree) && targetThree != null){
            return targetThree;
        } else {
            return null;
        }
    }
    private String getPathName(){
        return "/Users/Archish/Documents/Projects/MasqLib/FtcRobotController/src/main/assets/" + asset;
    }
}

package Library4997.MasqSensors;

/**
 * Created by Archish on 9/7/17.
 */

public class MasqCV {
    String targetOne, targetTwo, targetThree, asset;
    public MasqCV (String t1, String t2, String t3, String asset){
        targetOne = t1;
        targetTwo = t2;
        targetThree = t3;
        this.asset = asset;
    }
    public MasqCV (String t1, String t2, String asset){
        targetOne = t1;
        targetTwo = t2;
        targetThree = null;
        this.asset = asset;
    }
    public MasqCV (String t1, String asset){
        targetOne = t1;
        targetTwo = null;
        targetThree = null;
        this.asset = asset;
    }
    public int[] rbg (){
        int[] rbgreturn = new int[]{0};
        return rbgreturn;
    }
    public boolean isSeen(String target){
        return true;
    }
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
    private void findInAssets(){

    }
    private void pdf2jpeg(){

    }
}

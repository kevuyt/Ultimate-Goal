package Library4997.MasqExternal;

/**
 * Created by Archish on 10/16/17.
 */

public class MasqExternal {
    public static void sleep (int sleep) {
        try {Thread.sleep(sleep);}
        catch (InterruptedException e) {e.printStackTrace();}
    }
}

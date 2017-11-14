package Library4997.MasqOpenCV;

import android.graphics.Color;
import android.graphics.drawable.Drawable;
import android.hardware.Camera;
import android.view.View;

import org.opencv.android.OpenCVLoader;

/**
 * Created by Archish on 11/11/17.
 */

public class MasqOpenCV {
    public MasqOpenCV(){}
    private Drawable currentFrame;
    private Glyph glyph;
    private Cryptobox cryptobox;
    private Jewel jewel;
    public void getView(final View view){
        view.post(new Runnable() {
            public void run() {
                view.setBackground(currentFrame);
            }
        });
    }


}

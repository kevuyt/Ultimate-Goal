package com.disnodeteam.masqcv.detectors;

import com.disnodeteam.masqcv.OpenCVPipeline;

import org.opencv.core.Mat;

/**
 * Created by Victo on 12/17/2017.
 */

public class BlankDetector extends OpenCVPipeline {
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        return rgba;
    }
}

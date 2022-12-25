package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalDetectionPipeline extends OpenCvPipeline {
    Rect regionOfInterest = new Rect(0, 0, 0, 0);
    Mat region;
    Scalar color;

    public static volatile int val = 0;


    @Override
    public Mat processFrame(Mat input) {
        region = input.submat(regionOfInterest);

        color = Core.mean(region);

        // opencv colors are in BGR
        // 0 = yellow
        // 1 = magenta
        // 2 = cyan
        // you can change as needed
        if (color.val[0] < color.val[1] && color.val[0] < color.val[2]) val = 0;
        else if (color.val[1] < color.val[2]) val = 1;
        else val = 2;

        Imgproc.rectangle(input, regionOfInterest, new Scalar(255, 255, 255));

        return input;
    }

}

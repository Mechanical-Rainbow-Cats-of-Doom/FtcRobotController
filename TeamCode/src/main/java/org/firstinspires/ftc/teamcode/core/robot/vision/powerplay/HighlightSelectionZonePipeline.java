package org.firstinspires.ftc.teamcode.core.robot.vision.powerplay;

import org.firstinspires.ftc.teamcode.core.robot.vision.old.TsePipeline;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class HighlightSelectionZonePipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input) {
        ConePipeline.drawRectOnToMat(input, ConePipeline.getRect(), TsePipeline.yellow);
        return input;
    }
}

package org.firstinspires.ftc.teamcode.core.robot.vision.robot.powerplay;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/*
red
bottom height = 0.2
bottom width = 0.805
middle height = 0.3
middle width = 0.534
top height = 0.45
top width = 0.195

blue
bottom height = 0.41
bottom width = 0.75
middle height = 0.315
middle width = 0.37
top height 0.25
top width = 0.08
 */
@Config
public class ConePipeline extends OpenCvPipeline {
    public ConePipeline(boolean isRed) {
        if (isRed) {
            topRectHeightPercentage = 0.35D;
            topRectWidthPercentage = 0.2D;
        } else {
            topRectHeightPercentage = 0.2D;
            topRectWidthPercentage = 0.05D;
        }
    }
    public ConePipeline() {}


    //The position related to the screen
    public static double topRectWidthPercentage = 0.25;
    public static double topRectHeightPercentage = 0.50;
    //The points needed for the rectangles are calculated here
    public static int rectangleHeight = 10;
    //The width and height of the rectangles in terms of pixels
    public static int rectangleWidth = 10;
    private boolean running = false;

    public void startPipeline() {
        running = true;
    }
    public void stopPipeline() {
        running = false;
    }
    /**
     * @param input input frame matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        if (running) {
            Mat rgbMat = input.submat(new Rect(
                    (int) (input.width() * topRectWidthPercentage),
                    (int) (input.height() * topRectHeightPercentage),
                    rectangleWidth,
                    rectangleHeight
            ));
            Mat redMat = new Mat(), greenMat = new Mat(), blueMat = new Mat();
            Core.extractChannel(rgbMat, redMat, 0);
            Core.extractChannel(rgbMat, greenMat, 1);
            Core.extractChannel(rgbMat, blueMat, 2);

            double redMean = Core.mean(redMat).val[0], greenMean = Core.mean(greenMat).val[0], blueMean = Core.mean(blueMat).val[0];

        }
        return input;
    }



    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    private void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }

    /**
     * percentages of all rectangles. it goes top width, top height, middle width, etc.
     */
    public void setRectangles(double topRectWidthPercentage, double topRectHeightPercentage,
                              double middleRectWidthPercentage, double middleRectHeightPercentage,
                              double bottomRectWidthPercentage, double bottomRectHeightPercentage) {
        ConePipeline.topRectWidthPercentage = topRectWidthPercentage;
        ConePipeline.topRectHeightPercentage = topRectHeightPercentage;
    }
}

package org.firstinspires.ftc.teamcode.core.robot.vision.old.demo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline1Robot extends OpenCvPipeline {
    private Telemetry telemetry;
    //We declare the mats ontop so we can reuse them later to avoid memory leaks
    private Mat matYCrCb = new Mat();
    private Mat matCbBottom = new Mat();
    private Mat matCbTop = new Mat();
    private Mat topBlock = new Mat();
    private Mat bottomBlock = new Mat();

    //Where the average CB value of the rectangles are stored
    private double topAverage;
    private double bottomAverage;

    //The position related to the screen
    private double topRectWidthPercentage = 0.25;
    private double topRectHeightPercentage = 0.25;
    private double bottomRectWidthPercentage = 0.25;
    private double bottomRectHeightPercentage = 0.35;

    //The width and height of the rectangles in terms of pixels
    private int rectangleWidth = 10;
    private int rectangleHeight = 10;
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);
        telemetry.addData("Telemetry","Telemetry");
        telemetry.update();
        //The points needed for the rectangles are calculated here
        Rect topRect = new Rect(
                (int) (matYCrCb.width() * topRectWidthPercentage),
                (int) (matYCrCb.height() * topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        Rect bottomRect = new Rect(
                (int) (matYCrCb.width() * bottomRectWidthPercentage),
                (int) (matYCrCb.height() * bottomRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );

        //The rectangle is drawn into the mat
        drawRectOnToMat(input, topRect, new Scalar(255, 0, 0));
        drawRectOnToMat(input, bottomRect, new Scalar(100, 255, 0));

        //We crop the image so it is only everything inside the rectangles and find the cb value inside of them
        topBlock = matYCrCb.submat(topRect);
        bottomBlock = matYCrCb.submat(bottomRect);
        Core.extractChannel(bottomBlock, matCbBottom, 2);
        Core.extractChannel(topBlock, matCbTop, 2);

        //We take the average
        Scalar bottomMean = Core.mean(matCbBottom);
        Scalar topMean = Core.mean(matCbTop);

        bottomAverage = bottomMean.val[0];
        topAverage = topMean.val[0];

        //return the mat to be shown onto the screen
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

    public double getTopAverage() {
        return topAverage;
    }

    public double getBottomAverage() {
        return bottomAverage;
    }

    public void setTopRectWidthPercentage(double topRectWidthPercentage) {
        this.topRectWidthPercentage = topRectWidthPercentage;
    }

    public void setTopRectHeightPercentage(double topRectHeightPercentage) {
        this.topRectHeightPercentage = topRectHeightPercentage;
    }

    public void setBottomRectWidthPercentage(double bottomRectWidthPercentage) {
        this.bottomRectWidthPercentage = bottomRectWidthPercentage;
    }

    public void setBottomRectHeightPercentage(double bottomRectHeightPercentage) {
        this.bottomRectHeightPercentage = bottomRectHeightPercentage;
    }

    public void setRectangleWidth(int rectangleWidth) {
        this.rectangleWidth = rectangleWidth;
    }

    public void setRectangleHeight(int rectangleHeight) {
        this.rectangleHeight = rectangleHeight;
    }
}

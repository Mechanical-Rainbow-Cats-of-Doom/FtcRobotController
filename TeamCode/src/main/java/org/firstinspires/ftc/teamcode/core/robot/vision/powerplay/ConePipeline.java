package org.firstinspires.ftc.teamcode.core.robot.vision.powerplay;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.core.robot.vision.old.TsePipeline.yellow;

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
    public ConePipeline(boolean isRed, Telemetry telemetry, boolean debug) {
        if (isRed) {
            topRectHeightPercentage = 0.35D;
            topRectWidthPercentage = 0.2D;
        } else {
            topRectHeightPercentage = 0.2D;
            topRectWidthPercentage = 0.05D;
        }
        this.telemetry = telemetry;
        this.debug = debug;
    }
    public void reset() {
        running = true;
        finalPos = -1;
        totalTimesRan = 0;
        curRun = new Pair<>(-1 ,0);
        greatestConfidence = new Pair<>(-1, 0);
    }

    //The position related to the screen
    public static double topRectWidthPercentage = 0.25;
    public static double topRectHeightPercentage = 0.50;
    //The points needed for the rectangles are calculated here
    public static int rectangleHeight = 10;
    //The width and height of the rectangles in terms of pixels
    public static int rectangleWidth = 10;
    private boolean running = false;
    private final Telemetry telemetry;
    private int finalPos = -1; // 0 = pos1 (cyan), 1 = pos2(magneta), 2 = pos3(yellow)
    private Pair<Integer, Integer> curRun = new Pair<>(-1 ,0), greatestConfidence = new Pair<>(-1, 0);
    private int totalTimesRan = 0;
    private final boolean debug;
    public void startPipeline() {
        running = true;
    }

    public void interruptPipeline() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public synchronized Mat processFrame(Mat input) {
        if (running) {
            final Rect rect = new Rect(
                    (int) (input.width() * topRectWidthPercentage),
                    (int) (input.height() * topRectHeightPercentage),
                    rectangleWidth,
                    rectangleHeight
            );
            Mat rgbMat = input.submat(rect);
            if(debug) {
                drawRectOnToMat(input, rect, yellow);
            }

            Mat redMat = new Mat(), greenMat = new Mat(), blueMat = new Mat();
            Core.extractChannel(rgbMat, redMat, 0);
            Core.extractChannel(rgbMat, greenMat, 1);
            Core.extractChannel(rgbMat, blueMat, 2);
/*
            double[] cmykMean = rgbToCmyk(Core.mean(redMat).val[0], Core.mean(greenMat).val[0], Core.mean(blueMat).val[0]);
            telemetry.addData("red", cmykMean[0]);
            telemetry.addData("green", cmykMean[1]);
            telemetry.addData("blue", cmykMean[2]);
            final int pos = getIndexOfMaxOf3Params(cmykMean[0], cmykMean[1], cmykMean[2]);
*/
            final int pos = getIndexOfMaxOf3Params(Core.mean(redMat).val[0], Core.mean(greenMat).val[0], Core.mean(blueMat).val[0]);
            curRun = new Pair<>(pos, pos == curRun.first ? curRun.second + 1 : 0);
            if (curRun.second > greatestConfidence.second) {
                greatestConfidence = curRun;
            }
            totalTimesRan++;
            if (greatestConfidence.second >= 3 || totalTimesRan >= 6) {
                ConeDetector.visionVals.offer(greatestConfidence.first);
                running = false;
            }
        }
        return input;
    }

    @NonNull
    @Contract("_, _, _ -> new")
    public static double[] rgbToCmyk(double r, double g, double b) {
        double percentageR = r / 2.55; // r / 255 * 100
        double percentageG = g / 2.55;
        double percentageB = b / 2.55;
        double k = 100 - Math.max(Math.max(percentageR, percentageG), percentageB);
        if (k == 100) {
            return new double[] {0D, 0D, 0D, 100D};
        }
        return new double[]{
            (100 - percentageR - k) / (100 - k) * 100,
            (100 - percentageG - k) / (100 - k) * 100,
            (100 - percentageB - k) / (100 - k) * 100,
            k
        };
    }

    @NonNull
    @Contract("_ -> new")
    public static double[] rgbToCmyk(@NonNull double[] rgbArr) {
        return rgbToCmyk(rgbArr[0], rgbArr[1], rgbArr[2]);
    }

    public static int getIndexOfMaxOf3Params(double a, double b, double c) {
        double highMax = Math.max(Math.max(a, b), c);
        return (highMax == a ? 0 : (highMax == b ? 1 : 2 ));
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

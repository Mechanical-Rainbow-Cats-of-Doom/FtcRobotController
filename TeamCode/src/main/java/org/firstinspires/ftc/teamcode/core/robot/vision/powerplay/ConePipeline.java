package org.firstinspires.ftc.teamcode.core.robot.vision.powerplay;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmodes.tests.ConeVisionTester;
import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.ArrayBlockingQueue;

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
    public ConePipeline(boolean isRed, boolean debug, ArrayBlockingQueue<Integer> queue) {
        if (!debug && isRed) {
            topRectWidthPercentage = redWidthPercent;
            topRectHeightPercentage = redHeightPercent;
        }
        this.debug = debug;
        this.queue = queue;
    }

    public void reset() {
        running = true;
        totalTimesRan = 0;
        curRun = new Pair<>(-1, 0);
        greatestConfidence = new Pair<>(-1, 0);
    }

    //These are red
    public static double topRectWidthPercentage = 0.50;
    public static double topRectHeightPercentage = 0.35;
    public static double redWidthPercent = topRectWidthPercentage;
    public static double redHeightPercent = topRectHeightPercentage;
    public static double blueness = 2.1;
    //The points needed for the rectangles are calculated here
    public static int rectangleHeight = 10;
    //The width and height of the rectangles in terms of pixels
    public static int rectangleWidth = 10;
    private boolean running = false;
    private Pair<Integer, Integer> curRun = new Pair<>(-1, 0), greatestConfidence = new Pair<>(-1, 0);
    private final ArrayBlockingQueue<Integer> queue;
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

    @NonNull
    @Contract(value = " -> new", pure = true)
    public static Rect getRect() {
        return new Rect(
                (int) (ConeDetector.CAMERA_WIDTH * topRectWidthPercentage),
                (int) (ConeDetector.CAMERA_HEIGHT* topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public synchronized Mat processFrame(Mat input) {
        if (running) {
            final Rect rect = getRect();
            Mat rgbMat = input.submat(rect);
            drawRectOnToMat(input, rect, yellow);
            Mat redMat = new Mat(), greenMat = new Mat(), blueMat = new Mat();
            Core.extractChannel(rgbMat, redMat, 0);
            Core.extractChannel(rgbMat, greenMat, 1);
            Core.extractChannel(rgbMat, blueMat, 2);

            final double red = Core.mean(redMat).val[0];
            final double green = Core.mean(greenMat).val[0];
            final double blue = Core.mean(blueMat).val[0] * blueness;
            if (debug) {
                ConeVisionTester.setRedMean(red);
                ConeVisionTester.setGreenMean(green);
                ConeVisionTester.setBlueMean(blue);
            }
            final int pos = getIndexOfMaxOf3Params(red, green, blue);
            curRun = new Pair<>(pos, pos == curRun.first ? curRun.second + 1 : 0);
            if (curRun.second > greatestConfidence.second) {
                greatestConfidence = curRun;
            }
            totalTimesRan++;
            if (greatestConfidence.second >= 3 || totalTimesRan >= 6) {
                queue.offer(greatestConfidence.first);
                running = false;
            }
        }
        return input;
    }

    public static int getIndexOfMaxOf3Params(double a, double b, double c) {
        double highMax = Math.max(Math.max(a, b), c);
        return (highMax == a ? 0 : (highMax == b ? 1 : 2));
    }

    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    public static void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }
}

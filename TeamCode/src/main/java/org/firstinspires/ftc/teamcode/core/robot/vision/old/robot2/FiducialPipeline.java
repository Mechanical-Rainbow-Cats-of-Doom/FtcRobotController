package org.firstinspires.ftc.teamcode.core.robot.vision.old.robot2;

import android.graphics.BitmapFactory;

import boofcv.android.ConvertBitmap;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.ByteArrayInputStream;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.image.GrayF32;
import georegression.struct.point.Point2D_F64;
import static org.firstinspires.ftc.teamcode.core.robot.vision.old.robot2.FiducialDetector.CAMERA_WIDTH;
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
public class FiducialPipeline extends OpenCvPipeline {
    private final Mat markerImage = new Mat();
    private int location = -1;
    private boolean pipelineRunning = false;
    public void startPipeline() {
        pipelineRunning = true;
    }
    public void stopPipeline() {
        pipelineRunning = false;
    }

    public int getLocation() {
        return location;
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public Mat processFrame(Mat input) {
        if (pipelineRunning) {
            try {
                final MatOfByte mob = new MatOfByte();
                Imgcodecs.imencode(".bmp", input, mob);
                final GrayF32 original =ConvertBitmap.bitmapToGray(
                        BitmapFactory.decodeStream(new ByteArrayInputStream(mob.toArray())),
                        (GrayF32) null,
                        null);
                final FiducialDetector<GrayF32> detector = FactoryFiducial.squareBinary(new ConfigFiducialBinary(0.1), ConfigThreshold.local(ThresholdType.LOCAL_MEAN, 21), GrayF32.class);
                detector.detect(original);
                final Point2D_F64 locationPixel = new Point2D_F64();
                if (detector.totalFound() <= 1) {
                    return input;
                }
                detector.getCenter(0, locationPixel);
                //distance from 1st square to left side of screen, distance between spots
                final int pos = (int) Math.floor((locationPixel.getX() - (0.04 * CAMERA_WIDTH)) / (0.29 * CAMERA_WIDTH));
                if (pos >= 0 && pos <= 2) {
                    location = pos;
                    pipelineRunning = false;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        return input;
    }
}

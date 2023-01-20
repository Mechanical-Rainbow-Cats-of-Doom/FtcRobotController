package org.firstinspires.ftc.teamcode.core.robot.vision.powerplay;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.ArrayBlockingQueue;

public class ConeDetector {
    private final OpenCvCamera camera;
    private final boolean debug, isRed;
    private final Telemetry telemetry;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    public static ArrayBlockingQueue<Integer> visionVals = new ArrayBlockingQueue<>(1);
    public ConeDetector(HardwareMap hMap, String webcamName, boolean debug, boolean isRed, Telemetry telemetry) {
        this.isRed = isRed;
        this.debug = debug;
        this.telemetry = telemetry;
        OpenCvCameraFactory cameraFactory = OpenCvCameraFactory.getInstance();
        if (debug) {
            int cameraMonitorViewId = hMap
                    .appContext.getResources()
                    .getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());

            camera = cameraFactory.createWebcam(hMap.get(WebcamName.class, webcamName), cameraMonitorViewId); //for configurating remove isred from here
        } else {
            camera = cameraFactory.createWebcam(hMap.get(WebcamName.class, webcamName));
        }
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
                if (debug) {
                    FtcDashboard.getInstance().startCameraStream(camera, 10);
                }
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("OpenCv Pipeline error with error code " + errorCode);
            }
        });
    }

    /**
     * Resets pipeline on call
     * Stalls code until pipeline is done with figuring out (max time of around 0.33 seconds)
     *
     * @return integer 1 - 3, corresponds to cyan magenta or yellow
     */
    public int run() throws InterruptedException {
        final ConePipeline pipeline = new ConePipeline(isRed, telemetry, debug);
        camera.setPipeline(pipeline);
        pipeline.startPipeline();
        return visionVals.take();
    }
}

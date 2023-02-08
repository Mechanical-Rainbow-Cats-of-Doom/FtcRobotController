package org.firstinspires.ftc.teamcode.core.robot.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.jetbrains.annotations.Contract;

import java.util.Arrays;

import androidx.annotation.NonNull;

import static java.lang.Math.PI;

@Config
public class FunnyControllerMovement extends ControllerMovement {
    private boolean firstrun = true;
    public static int AVG_COUNT_ACCELERATING = 10, AVG_COUNT_DECELERATING = 5;
    private final BNO055IMU imu;
    private final Vector2d[] vals = new Vector2d[AVG_COUNT_ACCELERATING];
    private final ToggleableToggleButtonReader bReader;
    private int loopCount = 0;
    public FunnyControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        super(map, gamepad);
        bReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.B, true);
        drive.setPoseEstimate(PoseStorage.currentPose);
        imu = map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters() {{
            angleUnit = BNO055IMU.AngleUnit.RADIANS;
        }};
        imu.initialize(parameters);
    }
    @NonNull
    @Contract(" -> new")
    private Vector2d calcAvg() {
        double ytotal = 0, xtotal = 0;
        for (Vector2d vec : vals) {
            ytotal += vec.getY();
            xtotal += vec.getX();
        }
        return new Vector2d(xtotal / vals.length,ytotal / vals.length);
    }
    @SuppressWarnings("SuspiciousNameCombination")
    @Override
    public void update() {
        final double y = gamepad.getLeftY();
        final double x = -gamepad.getLeftX();
        final Vector2d raw = new Vector2d(y, x);
        if (firstrun) {
            Arrays.fill(vals, raw);
            firstrun = false;
        } else vals[Math.floorMod(loopCount, vals.length)] = raw;
            Vector2d avg = calcAvg();
        if (avg.getY() >= y + 0.2 || avg.getX() >= x + 0.2) {
            final int upper = Math.floorMod(loopCount, vals.length);
            final int lower = Math.floorMod((loopCount - AVG_COUNT_DECELERATING), vals.length);
            //try {
                if (lower < upper) Arrays.fill(vals, lower, upper, avg);
                else if (upper != 0) Arrays.fill(vals, upper - 1, lower - 1, avg);
                else Arrays.fill(vals, upper, vals.length, avg);
            //} catch (Exception e) {
             //   e.printStackTrace();
            //    Arrays.fill(vals, avg);
           // }
            avg = calcAvg();
        }
        bReader.readValue();
        if (bReader.getState()) {
            Vector2d input = new Vector2d(avg.getY(), avg.getX()).rotated(imu.getAngularOrientation().firstAngle - PI / 2);
            drive.setWeightedDrivePower(new Pose2d(
                    input.getX(),
                    input.getY(),
                    gamepad.getRightX()
            ));
            drive.updatePoseEstimate();
        } else super.update();
        if (Math.floorMod(++loopCount, 10) == 0) loopCount = 0;
    }
}

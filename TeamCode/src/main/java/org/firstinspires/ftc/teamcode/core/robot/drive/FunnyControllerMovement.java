package org.firstinspires.ftc.teamcode.core.robot.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;
import org.firstinspires.ftc.teamcode.core.softwaretools.CircularlyLinkedList;
import org.jetbrains.annotations.Contract;

import java.util.Arrays;
import java.util.Vector;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import static java.lang.Math.PI;
import static org.firstinspires.ftc.teamcode.core.softwaretools.CircularlyLinkedList.Node;

@Config
public class FunnyControllerMovement extends ControllerMovement {
    public static int AVG_COUNT_ACCELERATING = 4, AVG_COUNT_DECELERATING = 2;
    public static boolean inverseDrive = true, inverseStrafe = false, inverseRotate = false;
    private final BNO055IMU imu;
    private final CircularlyLinkedList<Vector2d> vals = new CircularlyLinkedList<>(AVG_COUNT_ACCELERATING, new Vector2d());
    private final ToggleableToggleButtonReader bReader;
    private final Telemetry telemetry;
    public FunnyControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad, @Nullable Telemetry telemetry) {
        super(map, gamepad);
        bReader = new ToggleableToggleButtonReader(gamepad, GamepadKeys.Button.B, true);
        drive.setPoseEstimate(PoseStorage.currentPose);
        imu = map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters() {{
            angleUnit = BNO055IMU.AngleUnit.RADIANS;
        }};
        imu.initialize(parameters);
        this.telemetry = telemetry;
    }

    public FunnyControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        this(map, gamepad, null);
    }

    @NonNull
    @Contract(" -> new")
    private Vector2d calcAvg() {
        double ytotal = 0, xtotal = 0;
        Node<Vector2d> node = vals.getHead();
        int itercount = 0;
        do {
            ytotal += node.getVal().getY();
            xtotal += node.getVal().getX();
            node = node.getNextNode();
            ++itercount;
        } while (node != vals.getHead());
        return new Vector2d(xtotal / itercount,ytotal / itercount);
    }

    @SuppressWarnings("SuspiciousNameCombination")
    @Override
    public void update() {
        final double y = gamepad.getLeftY();
        final double x = -gamepad.getLeftX();
        final Vector2d raw = new Vector2d(y, x);
        vals.add(raw);
        Vector2d avg = calcAvg();
        if (avg.getY() >= y + 0.2 || avg.getX() >= x + 0.2) {
            int itercount = 0;
            while (itercount++ < AVG_COUNT_DECELERATING) {
                vals.add(avg);
            }
            avg = calcAvg();
        }
        bReader.readValue();
        Vector2d input = new Vector2d(avg.getX() * (inverseStrafe ? -1 : 1), avg.getY() * (inverseDrive ? -1 : 1));
        if (!bReader.getState()) input = input.rotated(imu.getAngularOrientation().firstAngle - PI / 2);
        final double drivePow = input.getX();
        final double strafePow = input.getY();
        final double rotatePow = gamepad.getRightX() * (inverseRotate ? -1 : 1);
        drive.setWeightedDrivePower(new Pose2d(
                !bReader.getState() ? strafePow : drivePow,
                !bReader.getState() ? drivePow : strafePow,
                rotatePow
        ));
        drive.updatePoseEstimate();
        if (telemetry != null) {
            telemetry.addData("funnyDrive", drivePow);
            telemetry.addData("funnyStrafe", strafePow);
            telemetry.addData("funnyRotate", rotatePow);
        }
    }
}

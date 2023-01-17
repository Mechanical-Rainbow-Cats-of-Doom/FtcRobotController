package org.firstinspires.ftc.teamcode.core.robot.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;

import androidx.annotation.NonNull;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class FunnyControllerMovement extends ControllerMovement {
    private final Gamepad gamepad;
    public FunnyControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        super(map, gamepad);
        this.gamepad = gamepad.gamepad;
        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    @Override
    public void update() {
        //how roadrunner says to do it
        Vector2d input = new Vector2d(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x
        ).rotated(-drive.getPoseEstimate().getHeading());
        drive.setWeightedDrivePower(new Pose2d(
                input.getX(),
                input.getY(),
                -gamepad.right_stick_x
        ));
    }
}

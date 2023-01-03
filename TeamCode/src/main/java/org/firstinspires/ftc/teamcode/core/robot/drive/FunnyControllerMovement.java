package org.firstinspires.ftc.teamcode.core.robot.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;

import androidx.annotation.NonNull;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class FunnyControllerMovement extends ControllerMovement {
    public FunnyControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        super(map, gamepad);
        drive.setPoseEstimate(PoseStorage.currentPose);
    }

    @Override
    @SuppressWarnings("NonAsciiCharacters") // deal with it
    public void update() {
        double θ = drive.getPoseEstimate().getHeading();
        double S = gamepad.getLeftX();
        double D = gamepad.getLeftY();
        drive.setWeightedDrivePower(new Pose2d(
                S*cos(toRadians(90 - θ)) + D*sin(toRadians(90 - θ)),
                S*sin(toRadians(90 - θ)) + D*cos(toRadians(90 - θ)),
                gamepad.getRightX()
        ));
    }
}

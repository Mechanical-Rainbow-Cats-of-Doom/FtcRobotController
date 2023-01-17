package org.firstinspires.ftc.teamcode.core.robot.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class ControllerMovement {
    final SampleMecanumDrive drive;
    final GamepadEx gamepad;

    public ControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        this.drive = new SampleMecanumDrive(map);
        this.gamepad = gamepad;
    }

    public void update() {
        drive.setWeightedDrivePower(new Pose2d(gamepad.getLeftY(), -gamepad.getLeftX(), gamepad.getRightX()));
    }

    public ArrayList<Double> motorVelocities() {
        return (ArrayList<Double>) drive.getWheelVelocities();
    }
}

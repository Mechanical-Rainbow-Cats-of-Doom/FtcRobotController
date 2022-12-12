package org.firstinspires.ftc.teamcode.core.robot;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

public class ControllerMovement {
    private final GamepadEx gamepad;
    private final StrafingMovement move;

    public ControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        this.move = new StrafedMovementImpl(map);
        this.gamepad = gamepad;
    }

    public void update() {
        move.driveDRS(gamepad.getLeftY(), gamepad.getRightX(), gamepad.getLeftX());
    }

    public double[] motorVelocities() {
        return ((StrafedMovementImpl) move).motorVelocities();
    }
}

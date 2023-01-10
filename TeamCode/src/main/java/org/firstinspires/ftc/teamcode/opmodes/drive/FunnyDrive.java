package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.drive.FunnyControllerMovement;

@TeleOp
public class FunnyDrive extends NormalDrive {
    @Override
    FunnyControllerMovement createDrive(GamepadEx gamepad) {
        return new FunnyControllerMovement(hardwareMap, gamepad);
    }
}

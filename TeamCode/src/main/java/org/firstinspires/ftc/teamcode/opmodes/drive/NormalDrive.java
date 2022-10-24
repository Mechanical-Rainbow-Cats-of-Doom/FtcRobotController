package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;

@TeleOp
public class NormalDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final ControllerMovement drive = new ControllerMovement(hardwareMap, moveGamepad);
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
        }
    }
}
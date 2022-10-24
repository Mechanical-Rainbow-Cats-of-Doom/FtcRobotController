package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class NormalDrive extends LinearOpMode {
    @Override
    public void runOpMode() {
        final Encoder leftencoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final ControllerMovement drive = new ControllerMovement(hardwareMap, moveGamepad);
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("left encoder: ", leftencoder.getCurrentPosition());


        }
    }
}
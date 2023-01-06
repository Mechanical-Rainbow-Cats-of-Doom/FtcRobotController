package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.drive.ControllerMovement;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class NormalDrive extends LinearOpMode {
    final GamepadEx moveGamepad = new GamepadEx(gamepad1);
    ControllerMovement drive;

    ControllerMovement createDrive() {
        return new ControllerMovement(hardwareMap, moveGamepad);
    }

    @Override
    public void runOpMode() {
        drive = createDrive();
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("left encoder: ", leftEncoder.getCurrentPosition());
            telemetry.addData("right encoder: ", rightEncoder.getCurrentPosition());
            telemetry.addData("front encoder: ", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
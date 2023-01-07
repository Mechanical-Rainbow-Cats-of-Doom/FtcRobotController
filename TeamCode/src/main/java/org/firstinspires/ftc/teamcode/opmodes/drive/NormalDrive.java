package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.drive.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));

        telemetry.addLine("PATRICK I REMOVED THE LEP DELTA SORRY IF YOU WANTED THAT");
        telemetry.update();
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
            telemetry.addData("left encoder vel: ", leftEncoder.getCorrectedVelocity());
            telemetry.addData("right encoder vel: ", rightEncoder.getCorrectedVelocity());
            telemetry.addData("front encoder vel: ", frontEncoder.getCorrectedVelocity());
            telemetry.update();

        }
    }
}
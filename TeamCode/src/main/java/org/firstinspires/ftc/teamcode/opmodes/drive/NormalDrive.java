package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        int LEP = leftEncoder.getCurrentPosition();
        int REP = rightEncoder.getCurrentPosition();
        int FEP = frontEncoder.getCurrentPosition();

        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final ControllerMovement drive = new ControllerMovement(hardwareMap, moveGamepad);
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            telemetry.addData("left encoder pos: ", leftEncoder.getCurrentPosition() - LEP);
            telemetry.addData("right encoder pos: ", rightEncoder.getCurrentPosition() - REP);
            telemetry.addData("front encoder pos: ", frontEncoder.getCurrentPosition() - FEP);
            telemetry.addData("left encoder vel: ", leftEncoder.getCorrectedVelocity());
            telemetry.addData("right encoder vel: ", rightEncoder.getCorrectedVelocity());
            telemetry.addData("front encoder vel: ", frontEncoder.getCorrectedVelocity());
            telemetry.update();
        }
    }
}
package org.firstinspires.ftc.teamcode.opmodes.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.drive.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.TeleOpTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.TeleOpTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class NormalDrive extends LinearOpMode {
    ControllerMovement createDrive(GamepadEx gamepad) {
        return new ControllerMovement(hardwareMap, gamepad);
    }

    @Override
    public void runOpMode() {
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        final ControllerMovement drive = createDrive(moveGamepad);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        final TeleOpTools lift = new TeleOpTools(hardwareMap, new TeleOpTurret(hardwareMap, toolGamepad), toolGamepad, telemetry);
        telemetry.addLine("PATRICK I REMOVED THE LEP DELTA SORRY IF YOU WANTED THAT");
        telemetry.update();
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.FORWARD);
        frontEncoder.setDirection(Encoder.Direction.FORWARD);
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            lift.update();

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
package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.drive.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop.ControllerTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop.ControllerTurret;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;

import java.util.Timer;

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
        final ControllerTurret turret = new ControllerTurret(hardwareMap, toolGamepad);
        final ControllerTools tools = new ControllerTools(hardwareMap, new Timer(), turret, toolGamepad, telemetry, this);
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        tools.initIntake();
        while (opModeIsActive()) {
            drive.update();
            tools.update();
            telemetry.addData("Turret Rotation", turret.getPos(AutoTurret.Units.DEGREES));
            telemetry.addData("forward/backward: ", moveGamepad.getLeftY());
            telemetry.addData("left/right: ", -moveGamepad.getLeftX());
            telemetry.update();
        }
        tools.cleanup();
    }
}
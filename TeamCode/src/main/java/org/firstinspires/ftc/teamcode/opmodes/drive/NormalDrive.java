package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.drive.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop.ControllerTools;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Timer;

@TeleOp
public class NormalDrive extends LinearOpMode {
    protected MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
    protected ArrayDeque<LynxModule> hubs;
    ControllerMovement createDrive(GamepadEx gamepad) {
        return new ControllerMovement(hardwareMap, gamepad);
    }

    @Override
    public void runOpMode() {
        hubs = new ArrayDeque<>(Arrays.asList(PhotonCore.CONTROL_HUB, PhotonCore.EXPANSION_HUB));
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        final ButtonReader xButton = new ButtonReader(moveGamepad, GamepadKeys.Button.X);
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        int LEReset = leftEncoder.getCurrentPosition();
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        int REReset = rightEncoder.getCurrentPosition();
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        int FEReset = frontEncoder.getCurrentPosition();
        final ControllerMovement drive = createDrive(moveGamepad);
        final ControllerTools tools = new ControllerTools(hardwareMap, new Timer(), toolGamepad, moveGamepad, telemetry, this);
        final MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
            tools.update();
            telemetry.addData("left encoder: ", leftEncoder.getCurrentPosition()-LEReset);
            telemetry.addData("right encoder: ", rightEncoder.getCurrentPosition()-REReset);
            telemetry.addData("front encoder: ", frontEncoder.getCurrentPosition()-FEReset);
            telemetry.addData("Turret Rotation", tools.getTurretPos(AutoTurret.Units.DEGREES));
            telemetry.addData("forward/backward: ", moveGamepad.getLeftY());
            telemetry.addData("left/right: ", -moveGamepad.getLeftX());
            telemetry.update();
            hubs.forEach(LynxModule::clearBulkCache);
        }
    }
}
package org.firstinspires.ftc.teamcode.opmodes.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.util.ToggleableToggleButtonReader;

@TeleOp
public class AutoSetup extends LinearOpMode {
    public static void readButtons(@NonNull ButtonReader[] buttonReaders) {
        for (ButtonReader reader : buttonReaders) {
            reader.readValue();
        }
    }
    final GamepadEx configureGamepad = new GamepadEx(gamepad1);
    final MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    final ButtonReader yButton = new ButtonReader(configureGamepad, GamepadKeys.Button.Y);
    final ButtonReader xButton = new ButtonReader(configureGamepad, GamepadKeys.Button.X);
    final ButtonReader bButton = new ButtonReader(configureGamepad, GamepadKeys.Button.B);
    final ButtonReader aButton = new ButtonReader(configureGamepad, GamepadKeys.Button.A);
    final ButtonReader[] buttonReaders = {yButton, xButton, bButton, aButton};
    enum Setup {
        DELAY,
        SIDE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup window = Setup.DELAY;
        while (opModeIsActive()) {
            readButtons(buttonReaders);
            switch (window) {
                case DELAY:
                    telemetry.addLine("---Delay Setup---");
                    telemetry.addLine();
                    telemetry.addData();

                    break;

                case SIDE:

                    break;
            }
            telemetry.update();
        }
    }
}

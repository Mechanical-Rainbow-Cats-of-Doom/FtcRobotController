package org.firstinspires.ftc.teamcode.opmodes.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.util.AutoStorage;

@TeleOp
public class AutoSetup extends LinearOpMode {
    public static void readButtons(@NonNull ButtonReader[] buttonReaders) {
        for (ButtonReader reader : buttonReaders) {
            reader.readValue();
        }
    }
    final GamepadEx configureGamepad = new GamepadEx(gamepad1);
    final MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private final ButtonReader yButton = new ButtonReader(configureGamepad, GamepadKeys.Button.Y);
    private final ButtonReader xButton = new ButtonReader(configureGamepad, GamepadKeys.Button.X);
    private final ButtonReader bButton = new ButtonReader(configureGamepad, GamepadKeys.Button.B);
    private final ButtonReader aButton = new ButtonReader(configureGamepad, GamepadKeys.Button.A);
    private final ButtonReader next = new ButtonReader(configureGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    private final ButtonReader back = new ButtonReader(configureGamepad, GamepadKeys.Button.LEFT_BUMPER);
    private final ButtonReader[] buttonReaders = {yButton, xButton, bButton, aButton, next, back};
    enum Setup {
        DELAY, SIDE, CONES
    }

    private static final double deadzone = 0.1;
    private static final double timeBetweenContinuous = 500;
    private final ElapsedTime stickTimer = new ElapsedTime();
    private final ElapsedTime bumperTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        Setup window = Setup.DELAY;
        while (opModeIsActive()) {
            readButtons(buttonReaders);
            switch (window) {
                case DELAY:
                    telemetry.addLine("--Delay Setup---");
                    telemetry.addLine();
                    telemetry.addLine(Double.toString(AutoStorage.getDelay()));
                    telemetry.addLine("Up/Down on left stick to increase/decrease by 1");
                    telemetry.addLine("Left/Right on left stick to increase/decrease by .1");

                    if (configureGamepad.getLeftX() > deadzone) {
                        if (stickTimer.milliseconds() > timeBetweenContinuous) {
                            AutoStorage.addDelay(.1);
                            stickTimer.reset();
                        }
                    } else if (configureGamepad.getLeftX() < deadzone){
                        if(stickTimer.milliseconds() > timeBetweenContinuous) {
                            if (AutoStorage.getDelay() >= .1) {
                                AutoStorage.subtractDelay(.1);
                            } else {
                                AutoStorage.setDelay(0);
                            }
                            stickTimer.reset();
                        }
                    }

                    if (next.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.SIDE;
                    }
                    if (back.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.CONES;
                    }
                    break;
                case SIDE:


                    if (next.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.CONES;
                    }
                    if (back.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.DELAY;
                    }
                    break;
                case CONES:

                    if (next.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.DELAY;
                    }
                    if (back.isDown() && bumperTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.SIDE;
                    }
                    break;

            }
            telemetry.update();
        }
    }
}

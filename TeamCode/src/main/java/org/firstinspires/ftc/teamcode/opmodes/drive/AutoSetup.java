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
    // for navigating forwards and backwards
    private final ButtonReader next = new ButtonReader(configureGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    private final ButtonReader back = new ButtonReader(configureGamepad, GamepadKeys.Button.LEFT_BUMPER);
    private final ButtonReader[] buttonReaders = {yButton, xButton, bButton, aButton, next, back};
    enum Setup {
        DELAY, SIDE, CONES
    }

    private static final double deadzone = 0.5;
    private static final double timeBetweenContinuous = 100;
    private final ElapsedTime controlTimer = new ElapsedTime();
    private final ElapsedTime nextLastTimer = new ElapsedTime();
    private String lastPressed = "none";
    @Override
    public void runOpMode() throws InterruptedException {
        Setup window = Setup.DELAY;
        while (opModeIsActive()) {
            readButtons(buttonReaders);
            switch (window) {
                case DELAY:
                    telemetry.addLine("--Delay Setup---");
                    telemetry.addLine();
                    telemetry.addLine("Delay = " + AutoStorage.getDelay() + " seconds.");
                    telemetry.addLine("Up/Down on left stick to increase/decrease the delay by 1");
                    telemetry.addLine("Right/Left on left stick to increase/decrease the delay by .1");

                    if (configureGamepad.getLeftX() > deadzone) {
                        if (controlTimer.milliseconds() > timeBetweenContinuous || !lastPressed.equals("right")) {
                            AutoStorage.addDelay(.1);
                            controlTimer.reset();
                            lastPressed = "right";
                        }
                    } else if (configureGamepad.getLeftX() < deadzone){
                        if(controlTimer.milliseconds() > timeBetweenContinuous || !lastPressed.equals("left")) {
                            if (AutoStorage.getDelay() >= .1) {
                                AutoStorage.subtractDelay(.1);
                            } else {
                                AutoStorage.setDelay(0);
                            }
                            controlTimer.reset();
                            lastPressed = "left";
                        }
                    }

                    if (configureGamepad.getLeftY() > deadzone) {
                        if (controlTimer.milliseconds() > timeBetweenContinuous || !lastPressed.equals("up")) {
                            AutoStorage.addDelay(1);
                            controlTimer.reset();
                            lastPressed = "up";
                        }
                    } else if (configureGamepad.getLeftY() < deadzone){
                        if(controlTimer.milliseconds() > timeBetweenContinuous || !lastPressed.equals("down")) {
                            if (AutoStorage.getDelay() >= 1) {
                                AutoStorage.subtractDelay(1);
                            } else {
                                AutoStorage.setDelay(0);
                            }
                            controlTimer.reset();
                            lastPressed = "down";
                        }
                    }

                    if (next.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.SIDE;
                    }
                    if (back.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.CONES;
                    }
                    break;
                case SIDE:
                    telemetry.addLine("--Side Setup---");
                    telemetry.addLine();
                    telemetry.addLine("Current side: " + (AutoStorage.getSide() ? "Red" : "Blue"));
                    telemetry.addLine("Press A to switch the side.");

                    if(aButton.isDown() && (controlTimer.milliseconds() < timeBetweenContinuous || !lastPressed.equals("A"))) {
                        AutoStorage.changeSide();
                        lastPressed = "A";
                    }

                    if (next.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.CONES;
                    }
                    if (back.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.DELAY;
                    }
                    break;
                case CONES:
                    telemetry.addLine("--Cone Setup--");
                    telemetry.addLine();
                    telemetry.addLine("Cones: " + AutoStorage.getCones());
                    telemetry.addLine("Press A to add a cone, and B to remove a cone.");

                    if(aButton.isDown() && (controlTimer.milliseconds() < timeBetweenContinuous || !lastPressed.equals("A"))) {
                        AutoStorage.incrementCones();
                        lastPressed = "A";
                    } else if(bButton.isDown() && (controlTimer.milliseconds() < timeBetweenContinuous || !lastPressed.equals("B"))) {
                        AutoStorage.decrementCones();
                        lastPressed = "B";
                    }

                    if (next.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.DELAY;
                    }
                    if (back.isDown() && nextLastTimer.milliseconds() < timeBetweenContinuous) {
                        window = Setup.SIDE;
                    }
                    break;

            }
            telemetry.update();
        }
    }
}

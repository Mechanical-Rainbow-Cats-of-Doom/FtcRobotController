package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.shittyodometry.Movement;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.MyToggleButtonReader;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@TeleOp
public class NewRedDrive extends LinearOpMode {
    protected double power = -1;
    private final EventThread eventThread = new EventThread(() -> !isStopRequested());
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    BlueDrive.Mode currentMode = BlueDrive.Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        final Blinker[] lights = {
                hardwareMap.get(Blinker.class, "Control Hub"),
                hardwareMap.get(Blinker.class, "Expansion Hub 2")
        };
        final ArrayList<Blinker.Step> lightPattern = new ArrayList<>();
        lightPattern.add(new Blinker.Step(0xad2f, 150, TimeUnit.SECONDS));
        for (Blinker light : lights) {
            light.setPattern(lightPattern);
        }

        // Initialize custom cancelable SampleMecanumDrive class
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        final ControllerLift lift = new ControllerLift(eventThread, hardwareMap, toolGamepad, null);
        final Movement move = new Movement(hardwareMap, eventThread, moveGamepad, lift, power != 1);
        final SampleMecanumDrive drive = move.getDrive();
        final MyToggleButtonReader carouselButtonReader = new MyToggleButtonReader(toolGamepad, GamepadKeys.Button.B);
        final DcMotor carouselMotor = hardwareMap.get(DcMotor.class, "spinner");
        // will automatically run update method
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        Thread toolThread = new Thread(() -> {
            final ControllerIntake intake = new ControllerIntake(hardwareMap, eventThread, toolGamepad, power == 1);
            while (!isStopRequested()) {
                lift.update();
                intake.update(lift.getPosition());
            }
        });

        waitForStart();
        eventThread.start();
        toolThread.start();
        if (isStopRequested()) return;

        while (!isStopRequested()) {
            carouselButtonReader.readValue();
            if (carouselButtonReader.getState()) {
                carouselMotor.setPower(-0.6);
            } else {
                carouselMotor.setPower(0);
            }
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    move.update();
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (moveGamepad.getButton(GamepadKeys.Button.X)) {
                        drive.cancel();
                        currentMode = BlueDrive.Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = BlueDrive.Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
        lift.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        eventThread.interrupt();
        requestOpModeStop();
    }
}

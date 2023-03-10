package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConeDetector;
import org.firstinspires.ftc.teamcode.core.robot.vision.powerplay.ConePipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public abstract class AutoOpMode extends LinearOpMode {
    SampleMecanumDrive drive;
    AutoTurret turret;
    AutoTools tools;
    public static boolean isRed;
    ConeDetector detector;

    final ElapsedTime timer = new ElapsedTime();
    public void initialize() {

        drive = new SampleMecanumDrive(hardwareMap);
        turret = new AutoTurret(hardwareMap, 0);
        tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        Thread tool = new Thread(() -> {
            while (opModeIsActive()) {
                tools.update();
            }
        });
        detector = new ConeDetector(hardwareMap, "webcam", true, isRed);
        final Thread thread = new Thread(() -> {
            final GamepadEx moveGamepad = new GamepadEx(gamepad1);
            final ButtonReader up = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_UP);
            final ButtonReader left = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_LEFT);
            final ButtonReader down = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_DOWN);
            final ButtonReader right = new ButtonReader(moveGamepad, GamepadKeys.Button.DPAD_RIGHT);
            while (!isStarted()) {
                up.readValue();
                left.readValue();
                down.readValue();
                right.readValue();
                if (up.wasJustReleased()) ConePipeline.topRectHeightPercentage -= 0.01;
                if (down.wasJustReleased()) ConePipeline.topRectHeightPercentage += 0.01;
                if (left.wasJustReleased()) ConePipeline.topRectWidthPercentage -= 0.01;
                if (right.wasJustReleased()) ConePipeline.topRectWidthPercentage += 0.01;
            }
        });
        thread.start();
//        int color = detector.run();
        waitForStart();
        thread.interrupt();
        if (isStopRequested()) return;
        tool.start();

    }

    public void setStartingPos(double x, double y, double angle) {
        drive.setPoseEstimate(new Pose2d(x, y, Math.toRadians(angle)));
    }

    public void goForward(double inches) {

        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches)
                .build();

        drive.followTrajectory(trajectory);
    }

    public void goForward(double inches, double speed_multiplier) {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setConstraints(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return DriveConstants.MAX_VEL * speed_multiplier;
                    }
                }, new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return (speed_multiplier > 0.75 ? (DriveConstants.MAX_ACCEL * speed_multiplier) : 0.5);
                    }
                })
                .forward(inches)
                .resetConstraints()
                .build();
        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        drive.followTrajectorySequence(trajectory);

    }

    public void goBackward(double inches) {
        goForward(-inches);
    }

    public void goBackward(double inches, double speed_multiplier){
        goForward(-inches, speed_multiplier);
    }

    public void goLeft(double inches) {
        telemetry.addData("Going left: ", inches);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(inches)
                .build();
        telemetry.addLine("Trajectory built");
        drive.followTrajectory(trajectory);
    }

    public void goLeft(double inches, double speed_multiplier) {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setConstraints(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return DriveConstants.MAX_VEL * speed_multiplier;
                    }
                }, new TrajectoryAccelerationConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return (speed_multiplier > 0.7 ? (DriveConstants.MAX_ACCEL * /*speed_multiplier*/1) : 1);
                    }
                })
                .strafeLeft(inches)
                .resetConstraints()
                .build();
        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        drive.followTrajectorySequence(trajectory);

    }

    public void goRight(double inches) {
        goLeft(-inches);
    }

    public void goRight(double inches, double speed_multiplier) {
        goLeft(-inches, speed_multiplier);
    }


    public void turn(double degrees) {
        drive.turn(Math.toRadians(degrees));
    }

    public void wait(int milliseconds){
        timer.reset();
        while (timer.time(TimeUnit.MILLISECONDS) < milliseconds) {
            StayInPosition.stayInPose(drive, drive.getPoseEstimate());
        }
    }

}

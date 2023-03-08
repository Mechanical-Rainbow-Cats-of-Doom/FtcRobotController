package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Timer;

public abstract class AutoOpMode extends LinearOpMode {
    SampleMecanumDrive drive;
    AutoTurret turret;
    AutoTools tools;

    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        turret = new AutoTurret(hardwareMap, 0);
        tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        Thread thread = new Thread(() -> {
            while (opModeIsActive()) {
                tools.update();
            }
        });

        waitForStart();
        if (isStopRequested()) return;
        thread.start();
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

    public void goForward(double inches, double speed) {
        SampleMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(inches)
                .build();
        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        drive.followTrajectory(trajectory);
    }

    public void goBackward(double inches) {
        goForward(-inches);
    }

    public void goLeft(double inches) {
        telemetry.addData("Going left: ", inches);
        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(inches)
                .build();
        telemetry.addLine("Trajectory built");
        drive.followTrajectory(trajectory);
    }

    public void goRight(double inches) {
        goLeft(-inches);
    }


    public void turn(double degrees) {
        drive.turn(Math.toRadians(degrees));
    }

}

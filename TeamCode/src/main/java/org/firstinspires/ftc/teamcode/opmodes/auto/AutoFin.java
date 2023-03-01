package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Disabled
public class AutoFin extends LinearOpMode {
    boolean mirror = false;

    @Override
    public void runOpMode() throws InterruptedException {
        final Pose2d startPose = cMirrorY(new Pose2d(-35, 63,
                Math.toRadians(270)), mirror);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

        builder.strafeTo(cMirrorY(new Vector2d(-35, 12), mirror));
        builder.strafeTo(cMirrorY(new Vector2d(-22, 12), mirror));
        builder.waitSeconds(0.5);

        for (int i = 1; i <= 5; i++) {
            builder.strafeTo(cMirrorY(new Vector2d(-57, 12), mirror));
            builder.waitSeconds(0.5);
            builder.strafeTo(cMirrorY(new Vector2d(-22, 12), mirror));
            builder.waitSeconds(0.5);
        }

        TrajectorySequence sequence = builder.build();

        waitForStart();

        drive.followTrajectorySequence(sequence);
        while (!isStopRequested()) {
            drive.update();
        }
    }
}

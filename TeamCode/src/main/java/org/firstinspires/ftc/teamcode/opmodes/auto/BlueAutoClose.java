package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class BlueAutoClose extends LinearOpMode {
    public TrajectorySequence sequence;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(buildSequence(drive, isRed));
        waitForStart();

        drive.followTrajectorySequenceAsync(sequence);
        while(!isStopRequested()) {
            drive.update();
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        drive.followTrajectorySequenceAsync(null);
    }

    public Pose2d buildSequence(SampleMecanumDrive drive, boolean red) {
        final Pose2d initial = cMirrorY(new Pose2d(-35, 63, Math.toRadians(270)), red);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

        builder.strafeTo(cMirrorY(new Vector2d(-60, 58), red));
        for (int i = 0; i < 3; i++) {
            builder.strafeTo(cMirrorY(new Vector2d(-57, 13), red));
            builder.waitSeconds(0.5);
            builder.strafeTo(cMirrorY(new Vector2d(-22, 13), red));
            builder.waitSeconds(0.5);
        }

        sequence = builder.build();
        return initial;
    }
}

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class BlueAutoClose extends LinearOpMode {
    public TrajectorySequence sequence;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        // start pose
        final Pose2d startPose = cMirrorY(new Pose2d(-35, 63, Math.toRadians(270)), isRed);

        // build trajectory
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

        builder.strafeTo(cMirrorY(new Vector2d(-57, 13), isRed));
        builder.strafeTo(cMirrorY(new Vector2d(-22, 13), isRed));
        builder.waitSeconds(0.5);

        builder.strafeTo(cMirrorY(new Vector2d(-60, 58), isRed));
        for (int i = 0; i < 2; i++) {
            builder.strafeTo(cMirrorY(new Vector2d(-57, 13), isRed));
            builder.waitSeconds(0.5);
            builder.strafeTo(cMirrorY(new Vector2d(-22, 13), isRed));
            builder.waitSeconds(0.5);
        }

        sequence = builder.build();

        // set starting position
        drive.setPoseEstimate(startPose);
        // wait for the start button to be pressed
        waitForStart();
        // Run delay
        timer.reset();
        DelayStorage.waitForDelay(timer);

        //set to follow the sequence
        drive.followTrajectorySequenceAsync(sequence);
        // runs until the trajectory sequence is complete
        while(!isStopRequested() && drive.isBusy()) {
            // update the position
            drive.update();
            // keep the current position updated in
            PoseStorage.currentPose = drive.getPoseEstimate();
        }
        if (isStopRequested()) return;

        // after the trajectory sequence is complete
        while(!isStopRequested()) {
            StayInPosition.stayInPose(drive, sequence.end());
        }
    }
}

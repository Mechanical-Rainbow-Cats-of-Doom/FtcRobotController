package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

@Autonomous
@Disabled
public class AutoTest extends LinearOpMode {
    public TrajectorySequence sequence;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // start pose
        final Pose2d startPose = cMirrorY(new Pose2d(-35, 63, Math.toRadians(270)), isRed);

        // build trajectory
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

        builder.strafeTo(cMirrorY(new Vector2d(-60, 63), isRed));


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
            telemetry.addData("pose estimate", drive.getPoseEstimate());
            telemetry.update();
        }
        if (isStopRequested()) return;

        // after the trajectory sequence is complete
        while(!isStopRequested()) {
            StayInPosition.stayInPose(drive, sequence.end());
        }
    }
}

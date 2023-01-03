package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

@Autonomous
public class PatrickDriverOPmode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //set up
        TrajectorySequence sequence;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //defining the starting position
        final Pose2d startPose = cMirrorY(new Pose2d(-40,40), false);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

        //building the route
        builder.strafeTo(cMirrorY(new Vector2d(40, 30), false));
        sequence = builder.build();

        //setting the starting position
        drive.setPoseEstimate(startPose);

        //play is pressed
        waitForStart();

        //starting to follow the route
        drive.followTrajectorySequenceAsync(sequence);

        //if the program is running, loops while you are following the route
        while(!isStopRequested() && drive.isBusy()){
            // update roadrunner
            drive.update();

            //store your location to PoseStorage
            //for driver op
            PoseStorage.currentPose = drive.getPoseEstimate();
        }

        //checks if the program has been stopped, failsafe
        if(isStopRequested()) return;

        //loops if the program is not stopped but you have finished the route
        while(!isStopRequested()) {
            //forces the robot to maintain the last position of the route
            StayInPosition.stayInPose(drive, sequence.end());
        }
    }
}

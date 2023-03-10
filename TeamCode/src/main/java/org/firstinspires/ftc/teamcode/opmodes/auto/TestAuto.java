package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Timer;

@Autonomous
@Disabled

public class TestAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        int color = detector.run();
        setStartingPos(-35, 65, 180);

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        turret.setPos(-34, AutoTurret.Units.DEGREES);
        tools.setIntake(1);
        goLeft(3);

        //moving to the start requires a little more control
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(-25)
                .setConstraints((v, pose2d, pose2d1, pose2d2) -> DriveConstants.MAX_VEL * 0.8, (v, pose2d, pose2d1, pose2d2) -> (DriveConstants.MAX_ACCEL*0.9))
                .forward(-28.3)
                .resetConstraints()
                .build();
        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        drive.followTrajectorySequence(trajectory);
//      Not being used, look above
//        goBackward(15);
//        goBackward(39, 0.9);

        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);


        wait(1000);
        goRight(12, 0.74);
        wait(400);
        tools.setIntake(-1);
        wait(500);
        turret.setPos(54, AutoTurret.Units.DEGREES);
        wait(200);
        tools.setPosition(AutoTools.Position.HOVER_5);
        wait(100);

        goForward(2.8, 0.8); //slow this down
        goLeft(36.4);
        goBackward(1.50);

        tools.setIntake(1);
        wait(100);
        tools.setPosition(AutoTools.Position.INTAKE_5);
        wait(1000);
        tools.setPosition(AutoTools.Position.EXIT_5);
        wait(1000);

        goForward(1.50);
        goRight(20);

        wait(500);
        turret.setPos(-34, AutoTurret.Units.DEGREES);
        wait(200);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);
        goRight(15.5, 0.8);
        goBackward(3.2, 0.8);
        wait(500);
        tools.setIntake(-1);
        wait(500);


        switch (color){
            case 0:
                turret.setPos(-100, AutoTurret.Units.DEGREES);
                wait(1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(1000);
                turret.setPos(-200, AutoTurret.Units.DEGREES);
                goRight(12.5);
                break;
            case 1:
                turret.setPos(60, AutoTurret.Units.DEGREES);
                wait(1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(1000);
                turret.setPos(150, AutoTurret.Units.DEGREES);
                goLeft(12.5);
                break;
            case 2:
                turret.setPos(60, AutoTurret.Units.DEGREES);
                wait(1000);
                tools.setPosition(AutoTools.Position.HIGH_ARM);
                wait(1000);
                turret.setPos(150, AutoTurret.Units.DEGREES);
                goLeft(35);
                break;
        }

//        turret.setPos(54, AutoTurret.Units.DEGREES);
//        wait(100);
//        tools.setPosition(AutoTools.Position.HOVER_4);
//        wait(100);
//        goForward(2, 0.8);
//        wait(1000);
//        goLeft(35);
//        goBackward(1);
//        tools.setIntake(1);
//        tools.setPosition(AutoTools.Position.INTAKE_4);
//        wait(1000);
//        tools.setPosition(AutoTools.Position.EXIT_4);
//        wait(1000);
//        goRight(20);
//        wait(500);
//        turret.setPos(-34, AutoTurret.Units.DEGREES);
//        wait(200);
//        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);
//        goRight(15.5, 0.8);
//        goBackward(2.5, 0.8);
//        wait(500);
//        turret.setPos(-35, AutoTurret.Units.DEGREES);
//        tools.setIntake(-1);
//        wait(500);
//        turret.setPos(54, AutoTurret.Units.DEGREES);
//        wait(100);
//        tools.setPosition(AutoTools.Position.HOVER_3);
//        wait(1000);

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        wait(1000);



        tools.cleanup();
    }
}

package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.util.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.DelayStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.PoseStorage;
import org.firstinspires.ftc.teamcode.core.robot.util.StayInPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Timer;

@Autonomous

public class TestAuto extends AutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        setStartingPos(-35, 65, 180);

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        turret.setPos(-34, AutoTurret.Units.DEGREES);
        goLeft(3);
        goBackward(54);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);
        tools.setIntake(1);
        wait(1000);
        goRight(11, 0.5);
        wait(500);
        turret.setPos(-35, AutoTurret.Units.DEGREES);
        tools.setIntake(-1);
        wait(500);
        turret.setPos(54, AutoTurret.Units.DEGREES);
        wait(100);
        tools.setPosition(AutoTools.Position.HOVER_5);
        wait(100);
        goForward(5, 0.5); //slow this down
        wait(1000);
        goLeft(35.5);
        goBackward(1.25);
        tools.setIntake(1);
        tools.setPosition(AutoTools.Position.INTAKE_5);
        wait(1000);
        tools.setPosition(AutoTools.Position.EXIT_5);
        wait(1000);
        goRight(20);
        turret.setPos(-34, AutoTurret.Units.DEGREES);
        tools.setPosition(AutoTools.Position.HIGH_TARGET_LOWER);
        goRight(14.5, 0.5);
        goBackward(3.75, 0.5);
        wait(500);
        turret.setPos(-35, AutoTurret.Units.DEGREES);
        tools.setIntake(-1);
        wait(500);
        turret.setPos(54, AutoTurret.Units.DEGREES);
        wait(100);
        tools.setPosition(AutoTools.Position.HOVER_4);
        wait(100);
        goForward(5, 0.5);
        wait(1000);
        goLeft(35.5);
        goBackward(1.25);
        tools.setIntake(1);
        tools.setPosition(AutoTools.Position.INTAKE_4);
        wait(1000);
        tools.setPosition(AutoTools.Position.EXIT_4);
        wait(1000);
        goRight(20);

        tools.setPosition(AutoTools.Position.HIGH_ARM);
        wait(1000);



        tools.cleanup();
    }
}

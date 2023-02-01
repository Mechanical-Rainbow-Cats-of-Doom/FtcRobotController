package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.util.SewerOodoo;

@Autonomous
public class SewerOodooTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SewerOodoo odo = new SewerOodoo();

        waitForStart();

        odo.SetAxisMovement();
    }
}

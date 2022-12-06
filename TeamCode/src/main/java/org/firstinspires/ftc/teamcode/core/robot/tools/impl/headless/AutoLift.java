package org.firstinspires.ftc.teamcode.core.robot.tools.impl.headless;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class AutoLift {
    public DcMotorEx liftMotor;
    public DcMotorEx rotationMotor;

    enum Position {
        HOLDING(10, false),
        GROUND_TARGET(10, true),
        LOW_TARGET(100, true),
        MEDIUM_TARGET(200, true),
        HIGH_TARGET(300, true);

        final int motorPos;
        final boolean drop;

        Position(int motorPos, boolean drop) {
            this.motorPos = motorPos;
            this.drop = drop;
        }
    }

    Position position;
    /*
     * The stage represents how far the robot is from getting to the correct position.
     * 0 - initial position, hasn't started moving
     * 1 - started moving, hasn't reached destination
     * 2 - finished moving, hasn't optionally dumped
     * 3 - finished.
     * If the position doesn't dump, stage 2 is skipped.
     */
    int stage;

    public AutoLift(DcMotorEx liftMotor, DcMotorEx rotationMotor) {
        this.liftMotor = liftMotor;
        this.rotationMotor = rotationMotor;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
//        liftMotor.setTargetPosition(Math.abs(liftMotor.getCurrentPosition()));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void setPosition(Position position) {
        this.position = position;
        this.liftMotor.setTargetPosition(position.motorPos);
    }
}

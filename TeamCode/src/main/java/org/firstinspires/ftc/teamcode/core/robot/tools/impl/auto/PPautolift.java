package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PPautolift {
    public DcMotorEx liftMotor;
    public DcMotorEx rotationMotor;

    enum Position {
        INTAKE(10, false),
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

    Position position = Position.INTAKE;
    Position lastPosition = position;
    /*
     * The stage represents how far the robot is from getting to the correct position.
     * 0 - initial position, hasn't started moving
     * 1 - started moving, hasn't reached destination
     * 2 - finished moving, hasn't optionally dumped
     * 3 - finished.
     * If the position doesn't dump, stage 2 is skipped.
     */
    int stage = 0;
    int totalUpdates = 0;

    public PPautolift(DcMotorEx liftMotor, DcMotorEx rotationMotor) {
        this.liftMotor = liftMotor;
        this.rotationMotor = rotationMotor;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
//        liftMotor.setTargetPosition(Math.abs(liftMotor.getCurrentPosition()));
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);

    }

    public void setPosition(Position position) {
        this.position = position;
    }

    public void update() {
        if(lastPosition != position) {
            this.stage = 0;
            this.lastPosition = position;
        }
        switch (stage) {
            // initial position
            case 0:
                if(totalUpdates != 0) {
                    liftMotor.setTargetPosition(position.motorPos);
                }
                stage++;
                break;
            case 1:
                /*
                 * Do nothing for now, if we need to move out of the way of anything we can add it
                 * in later.
                 */
                 if(liftMotor.getCurrentPosition() == position.motorPos) {
                    stage++;
                 }
                 break;
            case 2:
                if(position.drop) {
                    // write code to drop here
                    break;
                }
            default:
                stage = 3;
            case 3:
                // youre done!
                break;
        }
        totalUpdates++;
    }
}
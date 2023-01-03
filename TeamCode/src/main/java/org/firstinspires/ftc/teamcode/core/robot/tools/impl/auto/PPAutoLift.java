package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

public class PPAutoLift {
    public final DcMotorEx liftMotor;
    private final Turret turret;

    public enum Position {
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

    public PPAutoLift(@NonNull DcMotorEx liftMotor, Turret turret) {
        this.liftMotor = liftMotor;
        this.turret = turret;
        ZeroMotorEncoder.zero(liftMotor, DcMotor.RunMode.RUN_TO_POSITION);
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
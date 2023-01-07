package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.tools.api.auto.AutoToggleableTool;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

public class AutoLift {
    final DcMotor liftMotor;
    final DcMotor armMotor;
    final Turret turret;
    final AutoToggleableTool<CRServo> intake;

    public enum Position { // THESE VALUES ARE JUST GUESSES
        NEUTRAL(40, 50),
        INTAKE(10, 25),
        GROUND_TARGET(10, 25),
        LOW_TARGET(100, 300),
        MEDIUM_TARGET(200, 500),
        HIGH_TARGET(300,800),
        MAX(10000, 1000); //armpos max is verified

        final int motorPos;
        final int armPos;

        Position(int motorPos, int armPos) {
            this.motorPos = motorPos;
            this.armPos = armPos;
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

    public AutoLift(@NonNull DcMotor liftMotor, @NonNull DcMotor armMotor, Turret turret, CRServo intakeServo) {
        this.liftMotor = liftMotor;
        this.armMotor = armMotor;
        this.turret = turret;
        this.intake = new AutoToggleableTool<>(intakeServo, 1);
        initMotors();
    }

    void initMotors() {
        ZeroMotorEncoder.zero(liftMotor);
        ZeroMotorEncoder.zero(armMotor);
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
                /*
                if(position.drop) {
                    // write code to drop here
                    break;
                }
                 */
            default:
                stage = 3;
            case 3:
                // youre done!
                break;
        }
        totalUpdates++;
    }
}
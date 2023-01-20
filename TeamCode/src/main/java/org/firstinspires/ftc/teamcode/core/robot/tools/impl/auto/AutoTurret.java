package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

public class AutoTurret {
    public enum Units {
        DEGREES,
        RADIANS,
        MOTOR_TICKS
    }

    public enum Rotation {
        FRONT(0),
        FRONTRIGHT(45),
        RIGHT(90),
        BACKRIGHT(135),
        BACK(180),
        BACKLEFT(225),
        LEFT(270),
        FRONTLEFT(315);

        final double motorPos;
        Rotation(double motorPos) {
            this.motorPos = motorPos * ticksperdeg;
        }
    }

    protected final DcMotor motor;
    // thanks ethan for the unlabeled magic numbers!
    // well i assume it was you but you were on logans computer or something
    public static final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5; // 5 for gear
    public static final double ticksperdeg = tpr / 360;

    protected void initMotors() {
        ZeroMotorEncoder.zero(motor);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public AutoTurret(@NonNull HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotor.class, "turret");
        motor.setZeroPowerBehavior(BRAKE);
        initMotors();
    }

    public boolean isMoving() {
        return motor.isBusy();
    }

    /**
     * sets the rotation of the tool in degrees, goes around if it would result in going through start pos
     *
     * @param unit give a unit type from this eunm {@link Units}
     */
    public void setPos(double pos, @NonNull Units unit) {
        switch (unit) {
            case RADIANS:
                pos = Math.toDegrees(pos);
            case DEGREES:
                pos *= ticksperdeg;
                break;
        }
        motor.setTargetPosition((int) Math.round(pos));
    }

    /**
     * don't call too often, relatively resource intensive
     *
     * @param unit give a unit type from this eunm {@link Units}
     * @return current pos in degrees
     */
    public double getPos(@NonNull Units unit) {
        double output = motor.getCurrentPosition();
        switch (unit) {
            case MOTOR_TICKS:
                return output;
            default:
                output /= ticksperdeg;
            case RADIANS:
                output = Math.toRadians(output);
                break;
        }
        return output;
    }
}
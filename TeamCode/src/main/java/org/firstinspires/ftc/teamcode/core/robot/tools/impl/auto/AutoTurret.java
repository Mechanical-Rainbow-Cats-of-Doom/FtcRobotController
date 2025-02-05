package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

@Config
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

        final double val;
        Rotation(double val) {
            this.val = val * ticksperdeg;
        }
    }

    protected final DcMotor motor;
    public static double tprmultiplier = 1;
    public static final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5 * tprmultiplier; // 5 for gear
    public static final double ticksperdeg = tpr / 360;
    private final double offset; // starting left corner
    protected void initMotors() {
        ZeroMotorEncoder.zero(motor, 1);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public AutoTurret(@NonNull HardwareMap hardwareMap, double offsetDeg) {
        this.motor = hardwareMap.get(DcMotor.class, "turret");
        this.offset = offsetDeg * ticksperdeg;
        motor.setZeroPowerBehavior(BRAKE);
        initMotors();
    }

    public AutoTurret(@NonNull HardwareMap hardwareMap, double offset, Units unit) {
        this(hardwareMap, unit == Units.RADIANS ? Math.toDegrees(offset) : unit == Units.MOTOR_TICKS ? offset / ticksperdeg : offset);
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

    public void setPos(@NonNull Rotation rotation) {
        setPos(rotation.val, Units.DEGREES);
    }

    /**
     * don't call too often, relatively resource intensive
     *
     * @param unit give a unit type from this eunm {@link Units}
     * @return current pos in degrees
     */
    public double getPos(@NonNull Units unit) {
        double output = motor.getCurrentPosition() + offset;
        if (unit == Units.MOTOR_TICKS) return output;
        else output /= ticksperdeg;
        if (unit == Units.RADIANS) output = Math.toRadians(output);
        return output;
    }

    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }

    public void cleanup() {
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
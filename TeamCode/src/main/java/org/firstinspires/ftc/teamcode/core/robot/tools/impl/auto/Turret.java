package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.core.robot.tools.api.ZeroMotorEncoder;
import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@Config
public class Turret {
    public static double homeDeg = 32;
    private final DcMotorEx motor;
    private final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5; // 5 for gear
    private final double ticksperdeg = tpr / 360;

    /**
     * Only run after init, robot crashes otherwise
     * @param motor turret motor
     */
    public Turret(@NonNull DcMotorEx motor) {
        this.motor = motor;
        motor.setZeroPowerBehavior(BRAKE);
        ZeroMotorEncoder.zero(motor);
    }

    /**
     * sets position of turret, goes around if it would result in going through start pos
     * @param pos degrees
     */
    public void setPosDeg(double pos) {
        pos += homeDeg;
        pos %= 360;
        motor.setTargetPosition((int) Math.round(pos * ticksperdeg));
    }

    /**
     * increments current position, rotates around if total set value results in >360
     * @param pos degrees
     */
    public void turnDeg(double pos) {
        setPosDeg(getPosDeg() + pos);
    }

    /**
     * don't call too often, relatively resource intensive
     * @return current pos in degrees
     */
    public double getPosDeg() {
        return motor.getCurrentPosition() / ticksperdeg;
    }
}

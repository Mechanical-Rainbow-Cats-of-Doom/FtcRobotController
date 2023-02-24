package org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.HashMap;
import java.util.LinkedHashMap;

@Config
public class ControllerTurret extends AutoTurret {
    private final GamepadEx gamepad, nihal;
    private final ControllerTools.BoxedBoolean doingstuff = new ControllerTools.BoxedBoolean();
    public static double ampltiude = 0.5;
    private final LinkedHashMap<ButtonReader, Double> turretButtons;
    private final HashMap<ButtonReader, Boolean> turretButtonVals = new HashMap<>();
    public final Rev2mDistanceSensorEx leftSensor, rightSensor;
    private boolean findingPole = false;
    private double poleFindingVelocity = 0;
    private final ButtonReader poleButtonLeft, poleButtonRight;
    @Override
    protected void initMotors() {
        ZeroMotorEncoder.zero(motor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public ControllerTurret(HardwareMap hardwareMap, GamepadEx gamepad, GamepadEx nihal, double offset) {
        super(hardwareMap, offset);
        this.gamepad = gamepad;
        this.nihal = nihal;
        leftSensor = hardwareMap.get(Rev2mDistanceSensorEx.class, "leftPoleDetector");
        leftSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);
        rightSensor = hardwareMap.get(Rev2mDistanceSensorEx.class, "rightPoleDetector");
        rightSensor.setRangingProfile(Rev2mDistanceSensorEx.RANGING_PROFILE.HIGH_SPEED);
        this.turretButtons = new LinkedHashMap<ButtonReader, Double>() {{
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_UP), 0D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_RIGHT), 90D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_DOWN), 180D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.DPAD_LEFT), 270D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.LEFT_BUMPER), 315D);
            put(new ButtonReader(gamepad, GamepadKeys.Button.RIGHT_BUMPER), 45D);
        }};
        this.poleButtonRight = new ButtonReader(gamepad, GamepadKeys.Button.A);
        this.poleButtonLeft = new ButtonReader(gamepad, GamepadKeys.Button.X);
    }

    public void whopper() {
        ControllerTools.setPosFromButtonMap(turretButtonVals, turretButtons, doingstuff, (turPos) -> {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            setPos(turPos, Units.DEGREES);
            findingPole = false;
        });
        final double neg = nihal.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        final double pos = nihal.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        poleButtonLeft.readValue();
        poleButtonRight.readValue();
        if (poleButtonLeft.wasJustPressed()) {
            findingPole = true;
            poleFindingVelocity = -1;
            motor.setPower(poleFindingVelocity);
        } else if (poleButtonRight.wasJustPressed()) {
            findingPole = true;
            poleFindingVelocity = 1;
            motor.setPower(poleFindingVelocity);
        }

        if (findingPole) {
            if (pos > 0.05 | neg > 0.05) {
                findingPole = false;
                return;
            }

            double distanceLeft = leftSensor.getDistance(DistanceUnit.CM);
            double distanceRight = rightSensor.getDistance(DistanceUnit.CM);
            boolean leftSeePole = distanceLeft < 50;
            boolean rightSeePole = distanceRight < 50;
            if (Math.abs(poleFindingVelocity) >= 1) {
                // stage 1: move until we see a pole
                if (leftSeePole | rightSeePole) {
                    poleFindingVelocity *= 0.5;
                }
            } else {
                // stage 2: move the turret more precisely to the exact position
                if (leftSeePole) {
                    if (rightSeePole) {
                        // done!
                        findingPole = false;
                        motor.setPower(0);
                        return;
                    }

                    // if we see only on the left that means we need to turn more to the left.
                    poleFindingVelocity = -Math.abs(poleFindingVelocity);
                } else {
                    if (!rightSeePole) {
                        // also done!
                        findingPole = false;
                        motor.setPower(0);
                        return;
                    }

                    // if we see only on the left that means we need to turn more to the right.
                    poleFindingVelocity = Math.abs(poleFindingVelocity);
                }
            }
            motor.setPower(poleFindingVelocity);

            return;
        }

        if (!doingstuff.value) {
            motor.setPower(Math.max(neg, pos) == neg ? -neg*ampltiude : pos*ampltiude);
        } else if (neg > 0.05 | pos > 0.05) {
            doingstuff.value = false;
            cleanup();
        }

    }
}

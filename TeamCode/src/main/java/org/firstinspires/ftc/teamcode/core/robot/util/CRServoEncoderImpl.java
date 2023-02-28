package org.firstinspires.ftc.teamcode.core.robot.util;
import com.qualcomm.robotcore.R;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * ContinuousRotationServoImpl provides an implementation of continuous
 * rotation servo functionality.
 */
public class CRServoEncoderImpl implements DcMotor {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected ServoController servoController = null;
    protected int servoPortNumber = -1;
    protected Direction servoDirection = Direction.FORWARD;

    protected static final double apiPowerMin = -1.0;
    protected static final double apiPowerMax =  1.0;
    protected static final double apiServoPositionMin = 0.0;
    protected static final double apiServoPositionMax = 1.0;

    //------------------------------------------------------------------------------------------------
    // Construction
    //------------------------------------------------------------------------------------------------




    //----------------------------------------------------------------------------------------------
    // HardwareDevice interface
    //----------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return servoController.getManufacturer();
    }

    @Override
    public String getDeviceName()
    {
        return AppUtil.getDefContext().getString(R.string.configTypeContinuousRotationServo);
    }

    @Override
    public String getConnectionInfo()
    {
        return servoController.getConnectionInfo() + "; port " + servoPortNumber;
    }

    @Override
    public int getVersion()
    {
        return 1;
    }

    @Override
    public synchronized void resetDeviceConfigurationForOpMode()
    {
        this.servoDirection = Direction.FORWARD;
    }

    @Override
    public void close()
    {
        // take no action
    }

    //----------------------------------------------------------------------------------------------
    // ContinuousRotationServo interface
    //----------------------------------------------------------------------------------------------
    /**
     * Constructor
     *
     * @param servoDirection  FORWARD for normal operation, REVERSE to reverse operation
     */
    public CRServoEncoderImpl(HardwareMap hardwareMap, String servoName, Direction servoDirection, String encoderName, Direction encoderDirection) {
        final CRServo servo = hardwareMap.get(CRServo.class, servoName);
        this.servoDirection = servoDirection;
        this.servoController = servo.getController();
        this.servoPortNumber = servo.getPortNumber();
        final DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        this.encoderPortNumber = encoder.getPortNumber();
        this.encoderController = encoder.getController();
        this.encoderDirection = encoderDirection;
        this.encoderMotorType = new MotorConfigurationType();
        encoderMotorType.setGearing(1);
        encoderMotorType.setMaxRPM(90);
        encoderMotorType.setTicksPerRev(4096);
        encoderMotorType.setAchieveableMaxRPMFraction(1);
    }

    protected DcMotorController            encoderController = null;
    protected int                          encoderPortNumber = -1;
    protected Direction                    encoderDirection  = Direction.FORWARD;
    protected MotorConfigurationType       encoderMotorType  = null;

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     *
     * @return the assigned type for this motor
     */
    @Override
    public MotorConfigurationType getMotorType() {
        return encoderController.getMotorType(servoPortNumber);
    }

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     *
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        encoderController.setMotorType(encoderPortNumber, motorType);
    }

    /**
     * Returns the underlying motor controller on which this motor is situated.
     *
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    @Override
    public DcMotorController getController() {
        return encoderController;
    }

    @Override
    public int getPortNumber()
    {
        return this.encoderPortNumber;
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     *
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see ZeroPowerBehavior
     * @see #setPower(double)
     */
    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        encoderController.setMotorZeroPowerBehavior(encoderPortNumber, zeroPowerBehavior);
    }

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     *
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return encoderController.getMotorZeroPowerBehavior(encoderPortNumber);
    }

    /**
     * Sets the zero power behavior of the motor to {@link ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE); // method not available in previous releases
     *     motor.setPowerFloat();
     *     motor.setPower(0.0);
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     * {@link #setZeroPowerBehavior(ZeroPowerBehavior) setZeroPowerBehavior()} and
     * {@link #setPower(double) setPower()}.
     */
    @Override
    public void setPowerFloat() {
        setZeroPowerBehavior(ZeroPowerBehavior.FLOAT);
        setPower(0D);
    }

    /**
     * Returns whether the motor is currently in a float power level.
     *
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    @Override
    public boolean getPowerFloat() {
        return getZeroPowerBehavior() == ZeroPowerBehavior.FLOAT && getPower() == 0.0;
    }

    protected Direction getOperationalDirection() {
        return encoderMotorType.getOrientation() == Rotation.CCW ? servoDirection.inverted() : servoDirection;
    }

    protected int adjustPosition(int position) {
        if (getOperationalDirection() == Direction.REVERSE) position = -position;
        return position;
    }

    protected void internalSetTargetPosition(int position) {
        encoderController.setMotorTargetPosition(encoderPortNumber, position);
    }

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(RunMode)
     * @see RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    @Override
    public void setTargetPosition(int position) {
        position = adjustPosition(position);
        internalSetTargetPosition(position);
    }

    /**
     * Returns the current target encoder position for this motor.
     *
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    @Override
    synchronized public int getTargetPosition() {
        int position = encoderController.getMotorTargetPosition(encoderPortNumber);
        return adjustPosition(position);
    }

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     *
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see RunMode#STOP_AND_RESET_ENCODER
     */
    @Override
    synchronized public int getCurrentPosition() {
        int position = encoderController.getMotorCurrentPosition(servoPortNumber);
        return adjustPosition(position);
    }

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     *
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    @Override
    public boolean isBusy() {
        return encoderController.isBusy(servoPortNumber);
    }

    protected void internalSetMode(RunMode mode) {
        encoderController.setMotorMode(encoderPortNumber, mode);
    }

    /**
     * Sets the current run mode for this motor
     *
     * @param mode the new current run mode for this motor
     * @see RunMode
     * @see #getMode()
     */
    @Override
    public void setMode(RunMode mode) {
        mode = mode.migrate();
        internalSetMode(mode);
    }

    /**
     * Returns the current run mode for this motor
     *
     * @return the current run mode for this motor
     * @see RunMode
     * @see #setMode(RunMode)
     */
    @Override
    public RunMode getMode() {
        return encoderController.getMotorMode(encoderPortNumber);
    }

    @Override
    public synchronized void setDirection(Direction direction)
    {
        this.servoDirection = direction;
    }

    @Override
    public synchronized Direction getDirection()
    {
        return this.servoDirection;
    }

    @Override
    public void setPower(double power)
    {
        // For CR Servos on MR/HiTechnic hardware, internal positions relate to speed as follows:
        //
        //      0   == full speed reverse
        //      128 == stopped
        //      255 == full speed forward
        //
        if (this.servoDirection == Direction.REVERSE) power = -power;
        power = Range.clip(power, apiPowerMin, apiPowerMax);
        power = Range.scale(power, apiPowerMin, apiPowerMax, apiServoPositionMin, apiServoPositionMax);
        this.servoController.setServoPosition(this.servoPortNumber, power);
    }

    @Override
    public double getPower()
    {
        double power = this.servoController.getServoPosition(this.servoPortNumber);
        power = Range.scale(power, apiServoPositionMin, apiServoPositionMax, apiPowerMin, apiPowerMax);
        if (this.servoDirection == Direction.REVERSE) power = -power;
        return power;
    }
}


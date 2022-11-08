package org.firstinspires.ftc.teamcode.core.robot.distance;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

//wiki for this device:
//https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304#target_8

@I2cDeviceType
@DeviceProperties(name = "SEN0304 Distance Sensor", description = "Distance Sensor from DF robot", xmlTag = "SEN0304")
public class SEN0304DistanceSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize() {
        this.writeShort(Register.Configure_Registers, (short) 0x00);
        return true;
    }

    @Override
    public String getDeviceName() {

        return "DFRobot SEN0304 Distance Sensor";
    }

    public void readDistance(){
        writeShort(Register.Command_Registers, (short) 0x01);
    }

    public int getDistance() {
        return readShort(Register.Distance_Value_HOBits);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public enum Register {
        FIRST(0),
        Product_ID(0x01),
        Version_Number(0x02),
        Distance_Value_HOBits(0x03),
        Distance_Value_LOBits(0x04),
        T_Value_HOBits(0x05),
        T_Value_LOBits(0x06),
        Configure_Registers(0x07),
        Command_Registers(0x08),
        LAST(Command_Registers.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }

    }

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);

    public SEN0304DistanceSensor(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }
}

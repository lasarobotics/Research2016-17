package com.qualcomm.ftcrobotcontroller.opmodes;
import android.provider.Telephony;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import navX.ftc.AHRS;
//Created by Ethan Schaffer for Demo Use.
 public class navXSimple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AHRS navx;
        String dimName = "dim";
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get(dimName);
        //you can name your device interface module dim,
        //or you can change dimName to "Device Interface Module 1"
        byte navXUpdateRate = 50;
        int navXPort = hardwareMap.i2cDevice.get("navX").getPort();
        navx = AHRS.getInstance(dim, navXPort, AHRS.DeviceDataType.kProcessedData, navXUpdateRate);
        waitForStart();
        double yaw;
        navx.zeroYaw();
        while(opModeIsActive()){
            waitOneFullHardwareCycle();
            yaw = navx.getYaw();
            telemetry.addData("Gyro", yaw);
            telemetry.addData("Firmware", navx.getFirmwareVersion());
        }
    }
}

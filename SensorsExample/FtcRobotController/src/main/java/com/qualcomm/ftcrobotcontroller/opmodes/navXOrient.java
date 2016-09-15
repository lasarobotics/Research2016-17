package com.qualcomm.ftcrobotcontroller.opmodes;

import android.provider.Telephony;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import navX.ftc.AHRS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Ethan Schaffer on 8/14/2016.
 */
public class navXOrient extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //general declarations
        DcMotor left1 = hardwareMap.dcMotor.get("l1"); // AL00XPH.port 1
        DcMotor left2 = hardwareMap.dcMotor.get("l2"); // AL00VEA7.port1
        DcMotor right1 = hardwareMap.dcMotor.get("r1");// AL00XPH.port 2
        DcMotor right2 = hardwareMap.dcMotor.get("r2"); // AL00VEA7.port2
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
        AHRS navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        navx.zeroYaw();

        double lPower = 0, rPower = 0;
        double yaw;

        waitForStart();

        while(opModeIsActive()){
            waitOneFullHardwareCycle();
            //get navX's yaw
            yaw = navx.getYaw();

            if(yaw>2.5){ //facing right
                rPower = yaw/15;
            } else if(yaw < -2.5){ //facing left
                rPower = (yaw)/15; //this will cause the right wheel to go backwards, which IS what we want
            } else {
                lPower = 0;
                rPower = 0;
            }
            left1.setPower(lPower);
            left2.setPower(lPower);
            right1.setPower(rPower);
            right2.setPower(rPower);
            telemetry.addData("yaw", yaw);
            telemetry.addData("l", lPower);
            telemetry.addData("r", rPower);

        }

    }
}

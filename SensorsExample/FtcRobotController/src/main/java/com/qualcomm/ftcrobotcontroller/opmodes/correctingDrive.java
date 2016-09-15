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
public class correctingDrive extends LinearOpMode {

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


        I2cDevice range = hardwareMap.i2cDevice.get("r");
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, range.getPort());

        double power = .1;
        int dist = 30; //number of centimeters away from object robot will stop
        int percentTolerance = 90; //if closer that this percent of distance, will back up
        double lPower = 0, rPower = 0, yaw;
        int cmAway;
        byte[] rangeReadings;

        waitForStart();
        navx.zeroYaw();
        while(opModeIsActive()){
            //range sensor handling
            rangeReadings = rangeReader.getReadBuffer();
            cmAway = rangeReadings[0] & 0xFF;
            if(cmAway > dist || cmAway < dist*percentTolerance/100){
                //get navX's gyro reading
                yaw = navx.getYaw();

                ///use the yaw to assign power to each wheel
                lPower = Range.clip(power - ((yaw) / 100), -1, 1);
                rPower = Range.clip(power + ((yaw)/100), -1, 1);

                //if we are too close, back up
                if(cmAway < dist*percentTolerance/100){
                    lPower = -lPower;
                    rPower = -rPower;
                }
            } else {
                lPower = 0;
                rPower = 0;
            }
            left1.setPower(lPower);
            left2.setPower(lPower);
            right1.setPower(rPower);
            right2.setPower(rPower);
            telemetry.addData("Cm from wall", cmAway);
            telemetry.addData("NavX Yaw", navx.getYaw());
            telemetry.addData("Left Power", lPower);
            telemetry.addData("Right Power", rPower);
            waitOneFullHardwareCycle();
        }

    }
}

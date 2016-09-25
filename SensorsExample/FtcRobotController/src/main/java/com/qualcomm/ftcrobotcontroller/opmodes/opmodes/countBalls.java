package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

/**
 * Created by Ethan Schaffer on 8/14/2016.
 */
public class countBalls extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        I2cDevice range = hardwareMap.i2cDevice.get("r"); // dim.port2
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
        byte[] rangeReadings;
        int closeness;
        int count = 0;
        boolean ballLastSecond = false;
        int fullCount = 0;
        waitForStart();
        rangeReadings = rangeReader.getReadBuffer();
        int threshold = rangeReadings[1] & 0xFF;
        while(opModeIsActive()){
            rangeReadings = rangeReader.getReadBuffer();
            closeness = rangeReadings[1] & 0xFF;
            telemetry.addData("Closeness", closeness);
            telemetry.addData("Balls", count);
            telemetry.addData("Threshold", threshold);
            if(closeness > threshold && !ballLastSecond){
                count++;
                ballLastSecond = true;
            } else if(closeness <= threshold){
                ballLastSecond = false;
            }
            if(closeness > threshold && ballLastSecond){
                fullCount++;
                if(fullCount > 10000)
                    telemetry.addData("Balls", "Full");
            } else {
                fullCount = 0;
            }
        }
    }
}

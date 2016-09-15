package com.qualcomm.ftcrobotcontroller.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
/**
 * Created by Ethan Schaffer on 8/14/2016.
 */
public class range extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        I2cDevice range = hardwareMap.i2cDevice.get("r"); // dim.port2
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
        byte[] rangeReadings;
        int cmAway, closeness;
        waitForStart();
        while(opModeIsActive()){
            rangeReadings = rangeReader.getReadBuffer();
            cmAway = rangeReadings[0] & 0xFF;
            closeness = rangeReadings[1] & 0xFF;
            telemetry.addData("Cm", cmAway);
            telemetry.addData("closeness", closeness);
        }
    }
}

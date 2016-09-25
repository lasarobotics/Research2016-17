package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

//Created by Ethan Schaffer, for Demo Use
public class colorPassiveMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.colorSensor.get("c"); // dim.port1
        waitForStart();
        boolean ledOn = false;
        while(opModeIsActive()){
            color.enableLed(ledOn); //sets up active mode or passive mode (solid objects vs beacons)
//            telemetry.addData("Clear", color.alpha());
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            if(color.blue() > color.red() && color.blue() > color.green())
                telemetry.addData("Color", "Blue!");
            else if(color.red() > color.blue() && color.red() > color.green())
                telemetry.addData("Color", "Red!");
            else if(color.green() > color.red() && color.green() > color.blue())
                telemetry.addData("Color", "Green!");
            else
                telemetry.addData("Color", "Not Sure... :(");
            waitOneFullHardwareCycle();
        }
    }
}

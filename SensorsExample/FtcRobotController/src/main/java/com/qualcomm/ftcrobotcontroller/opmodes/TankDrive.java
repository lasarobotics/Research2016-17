package com.qualcomm.ftcrobotcontroller.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Created by Ethan Schaffer on 4/29/2016.
 */
public class TankDrive extends OpMode{
    DcMotor left1, right1, left2, right2;
    I2cDevice range;
    ColorSensor color;
    DeviceInterfaceModule CDI;


    /*
    Setting to 50 Hertz is better,
    because that is the light frequency
    used by the US and other
    North American countries.
    */
    boolean distOn = true;
    boolean colorOn = false;

    I2cDeviceReader rangeReader;
    byte rangeReadings[];

    @Override
    public void init() {
        left1 = hardwareMap.dcMotor.get("l1"); // AL00XPH.port1
        left2 = hardwareMap.dcMotor.get("l2"); // AL00VEA7.port1
        right1 = hardwareMap.dcMotor.get("r1"); // AL00XPH.port2
        right2 = hardwareMap.dcMotor.get("r2"); // AL00VEA7.port2
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        color = hardwareMap.colorSensor.get("c"); // dim.port1
        range = hardwareMap.i2cDevice.get("r"); // dim.port2
        CDI = hardwareMap.deviceInterfaceModule.get("dim");
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
    }

    @Override
    public void loop(){
        /*
        * RANGE SENSOR
        * */

        if(distOn) {
            rangeReadings = rangeReader.getReadBuffer();
            telemetry.addData("rangeReadings[0]", rangeReadings[0]);
            telemetry.addData("rangeReadings[1]", rangeReadings[1]);

            telemetry.addData("Ultrasonic (Far)", (rangeReadings[0] & 0xFF));
            telemetry.addData("Optical (Near)", (rangeReadings[1] & 0xFF));
        } else {
            telemetry.addData("Range Reader", "Is Off");
        }

        /*
        * COLOR SENSOR
        * */
        float hsvValues[] = {0, 0, 0};
        if(colorOn) {
            color.enableLed(true);
            /*
            The above sets the sensor in
            either active or passive mode
            Active Mode is used for
            solid objects, like plastic balls

            Passive Mode is used for
            lights, such as the resQ beacon
            */
            if (gamepad1.a)
                color.enableLed(false);
            else
                color.enableLed(true);
            telemetry.addData("Clear", color.alpha());
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
            telemetry.addData("HSV", hsvValues[0]);
            if(color.blue() > color.red() && color.blue() > color.green())
                telemetry.addData("Color", "Blue!");
            else if(color.red() > color.blue() && color.red() > color.green())
                telemetry.addData("Color", "Red!");
            else if(color.green() > color.red() && color.green() > color.blue())
                telemetry.addData("Color", "Green!");
            else
                telemetry.addData("Color", "Not Sure... :(");
        } else {
            telemetry.addData("Color", "Is Off");
        }

        int mod = 1;
        if(gamepad1.right_bumper){
            mod = 3;
        }
        if(Math.abs(gamepad1.left_stick_y) > .1 ){
            left1.setPower(gamepad1.left_stick_y / mod);
            left2.setPower(gamepad1.left_stick_y / mod);
        } else {
            left1.setPower(0);
            left2.setPower(0);
        }
        if(Math.abs(gamepad1.right_stick_y) > .1 ){
            right1.setPower(gamepad1.right_stick_y / mod);
            right2.setPower(gamepad1.right_stick_y / mod);
        } else {
            right1.setPower(0);
            right2.setPower(0);
        }

        /*
        * Device Interface Module
        */
        CDI.setLED(0, true); //Blue LED
        CDI.setLED(1, true); //Red LED

    }

    @Override
    public void stop() {

    }
}
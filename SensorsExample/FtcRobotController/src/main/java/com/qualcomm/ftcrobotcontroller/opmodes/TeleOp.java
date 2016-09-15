package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;
import com.qualcomm.robotcore.robocol.Telemetry;

import navX.AHRSProtocol;
import navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 8/16/2016.
 */
public class TeleOp extends OpMode {
    DcMotor left1, left2, right1, right2;
    AHRS navx;
    ColorSensor color;
    I2cDevice range;
    I2cDeviceReader rangeReader;
    public String getColor(int red, int blue, int green){
        if(red > blue && red > green)
            return "red";
        if(blue>red && blue > green)
            return "blue";
        if(green > red && green > blue)
            return "green";
        return "no result";
    }

    @Override
    public void init() {
        //general declarations
        left1 = hardwareMap.dcMotor.get("l1"); // AL00XPH.port 1
        left2 = hardwareMap.dcMotor.get("l2"); // AL00VEA7.port1
        right1 = hardwareMap.dcMotor.get("r1");// AL00XPH.port 2
        right2 = hardwareMap.dcMotor.get("r2"); // AL00VEA7.port2
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        left1.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        left2.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        right1.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        right2.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(left1.getCurrentPosition() != 0){}
        left1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        left2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        right1.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        right2.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
        navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        navx.zeroYaw();

        color = hardwareMap.colorSensor.get("c");
        color.enableLed(true);

        range = hardwareMap.i2cDevice.get("r");
        rangeReader = new I2cDeviceReader(range, 0x28, 0x04, range.getPort());

    }

    @Override
    public void loop() {
        //range sensor handling
        byte[] rangeReadings = rangeReader.getReadBuffer();
        int cmAway = rangeReadings[0] & 0xFF;
        telemetry.addData("Closeness", rangeReadings[1] & 0xFF);
        telemetry.addData("Cm away", cmAway);
//        telemetry.addData("Yaw", navx.getYaw());
        telemetry.addData("Color", getColor(color.red(), color.blue(), color.green()));
        telemetry.addData("Alpha", color.alpha());
//        telemetry.addData("Left_Encoder_Average", (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2);
//        telemetry.addData("Right_Encoder_Average", (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2);


        double leftPower = gamepad1.left_stick_y, rightPower = gamepad1.right_stick_y;
        if(Math.abs(leftPower) < .05)
            leftPower = 0;
        if(Math.abs(rightPower) < .05)
            rightPower = 0;
        left1.setPower(leftPower);
        left2.setPower(leftPower);
        right1.setPower(rightPower);
        right2.setPower(rightPower);

    }
}

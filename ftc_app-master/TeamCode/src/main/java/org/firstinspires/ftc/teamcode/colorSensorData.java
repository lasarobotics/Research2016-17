package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="ColorSensorFinal", group="Autonomous")
public class colorSensorData extends LinearOpMode{

    ColorSensor color1, color2;
    @Override
    public void runOpMode() throws InterruptedException {
        color1 = hardwareMap.colorSensor.get("cb");
//        color1.setI2cAddress(I2cAddr.create8bit(0x4c));
        color2 = hardwareMap.colorSensor.get("cs");
        color2.setI2cAddress(I2cAddr.create8bit(0x3c));
        color1.enableLed(true);
        color2.enableLed(false);

        waitForStart();
        while(opModeIsActive()){
            color2.setI2cAddress(I2cAddr.create8bit(0x3c));
            color1.enableLed(true);
            color2.enableLed(false);
            telemetry.addData("color1.red()", color1.red());
            telemetry.addData("color1.green()", color1.green());
            telemetry.addData("color1.blue()", color1.blue());
            telemetry.addData("-----", "------");
            telemetry.addData("color2.red()", color2.red());
            telemetry.addData("color2.green()", color2.green());
            telemetry.addData("color2.blue()", color2.blue());

            telemetry.update();
            idle();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="ColorSensorFinal", group="Autonomous")
public class colorSensorData extends LinearOpMode{

    ColorSensor color_bottom, color_side;
    @Override
    public void runOpMode() throws InterruptedException {
        color_bottom = hardwareMap.colorSensor.get("cb");
        color_bottom.setI2cAddress(I2cAddr.create8bit(0x4c));
        color_side = hardwareMap.colorSensor.get("cs");
        color_side.setI2cAddress(I2cAddr.create8bit(0x3c));
        color_bottom.enableLed(true);
        color_side.enableLed(false);

        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("bottom red", color_bottom.red());
            telemetry.addData("bottom green", color_bottom.green());
            telemetry.addData("bottom blue", color_bottom.blue());
            telemetry.addData("bottom alpha", color_bottom.alpha());
            telemetry.addData("-----", "------");
            telemetry.addData("side red", color_side.red());
            telemetry.addData("side green", color_side.green());
            telemetry.addData("side blue", color_side.blue());

            telemetry.update();
            idle();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Ethan Schaffer on 10/6/2016.
 */
@Autonomous(name="C_T", group="Autonomous")
public class colorSensorData extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.get(ColorSensor.class, "c");
        waitForStart();
        color.enableLed(true);
        while(opModeIsActive()){
            telemetry.addData("i2c", color.getI2cAddress());
            telemetry.addData("R", color.red());
            telemetry.addData("arbg", color.argb());
            telemetry.addData("G", color.green());
            telemetry.addData("B", color.blue());
            telemetry.update();
        }
    }
}

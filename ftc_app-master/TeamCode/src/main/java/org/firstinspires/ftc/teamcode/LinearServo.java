package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ethan Schaffer on 11/5/2016.
 */
@TeleOp(name="Linear Servo", group="TeleOp")
public class LinearServo extends OpMode {
    Servo linear;

    @Override
    public void init() {
        linear = hardwareMap.servo.get("l");
    }

    @Override
    public void loop() {
        double value = gamepad1.left_stick_y/2+.5;
        linear.setPosition(value);
        telemetry.addData("Value", linear.getPosition());

    }
}

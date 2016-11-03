/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name="Final Auto (L)", group="Autonomous")
public class AutoMecanumFinal extends LinearOpMode {
    //TWEAKING VALUES
    public static final double BLOCKSERVOOPENVALUE = 0;
    public static final double BLOCKSERVOCLOSEDVALUE = 1;
    public static final double MAXINFEEDPOWER = 1;
    public static final double MAXOUTFEEDPOWER = -1;
    public static final double INFEEDOFFPOWER = 0;
    public static final double SERVOINCREMENTVALUE = .02;
    public static final double LEFTSERVOMAXVALUE = 1;
    public static final double LEFTSERVOMINVALUE = 0;
    public static final double RIGHTSERVOMAXVALUE = 1;
    public static final double RIGHTSERVOMINVALUE = 0;

    //GOOD VALUES
    public enum GOING {
        IN, OUT, NOTGOING
    }
    public TeleOpFinal.GOING INFEEDSTATUS = TeleOpFinal.GOING.NOTGOING;
    public boolean RECENTLEFTBUMPERVALUE = false;
    public boolean RECENTRIGHTBUMPERVALUE = false;
    public static final double SLOWDOWNVALUE = 5;
    public static final double TRIGGERTHRESHOLD = .2;
    public static final double ACCEPTINPUTTHRESHOLD = .15;
    public static final double SHOOTERMAXVALUE = 1;
    public static final double SHOOTERMINVALUE = 0;
    public static final double SHOOTERTHRESHOLD = .2;
    public static final double SHOOTERLOWERRATE = 0;
    //I know these should be switched, but the hardware map is a pain to fix.
    public static final String LEFT1NAME = "l2";
    public static final String LEFT2NAME = "l1";
    //I know these should be switched, but the hardware map is a pain to fix.
    public static final String RIGHT1NAME = "r1";
    public static final String RIGHT2NAME = "r2";
    public static final String SHOOT1NAME = "sh1";
    public static final String SHOOT2NAME = "sh2";
    public static final String INFEEDNAME = "in";
    public static final String BALLBLOCKNAME = "b";
    public static final String LEFTPUSHNAME = "lp";
    public static final String RIGHTPUSHNAME = "rp";
    public static final String RANGENAME = "r";
    public static final String COLORSIDENAME = "cs";
    public static final String COLORBOTTOMNAME = "cb";


    DcMotor left1, left2, right1, right2, shoot1, shoot2, infeed;
    Servo leftPush, rightPush, ballBlock;
    ColorSensor color1, color2;
    ModernRoboticsI2cRangeSensor range;

    //Runs op mode
    @Override
    public void runOpMode() throws InterruptedException {

        left1 =hardwareMap.dcMotor.get(LEFT1NAME);
        left2 =hardwareMap.dcMotor.get(LEFT2NAME);
        right1=hardwareMap.dcMotor.get(RIGHT1NAME);
        right2=hardwareMap.dcMotor.get(RIGHT2NAME);
        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot1=hardwareMap.dcMotor.get(SHOOT1NAME);
        shoot1.setDirection(DcMotorSimple.Direction.REVERSE);
        shoot2=hardwareMap.dcMotor.get(SHOOT2NAME);
        infeed=hardwareMap.dcMotor.get(INFEEDNAME);
        infeed.setDirection(DcMotorSimple.Direction.REVERSE);
        ballBlock=hardwareMap.servo.get(BALLBLOCKNAME);
        leftPush =hardwareMap.servo.get(LEFTPUSHNAME);
        rightPush=hardwareMap.servo.get(RIGHTPUSHNAME);
        leftPush.setPosition(LEFTSERVOMAXVALUE);
        rightPush.setPosition(RIGHTSERVOMINVALUE);
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, RANGENAME);
        color1 = hardwareMap.colorSensor.get(COLORBOTTOMNAME);
        color2 = hardwareMap.colorSensor.get(COLORSIDENAME);
        telemetry.addData("raw ultrasonic", range.rawUltrasonic());
        telemetry.update();

        // wait for the start button to be pressed
//        double startingG = gyroSensor.getHeading();

        left1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        while(left1.getCurrentPosition()!=0){
        }//wait
        int encPos = left1.getCurrentPosition();

        //WAIT FOR START IS HERE
        waitForStart();

        resetEncs(left1, right1, left2, right2);
        telemetry.addData("Left", encPos);
        telemetry.update();
        drive(-.85, left1, right1, left2, right2);
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 1000) {
            telemetry.addData("Left", left1.getCurrentPosition() - encPos);
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);
        ballBlock.setPosition(BLOCKSERVOOPENVALUE); //Make sure to update
        shoot1.setPower(1);
        shoot2.setPower(1);
        sleep(1000);
        shoot1.setPower(0);
        shoot2.setPower(0);
        ballBlock.setPosition(BLOCKSERVOCLOSEDVALUE); //Make sure to update
        drive(-.85, left1, right1, left2, right2);
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 1500) {
            telemetry.addData("Left", left1.getCurrentPosition() - encPos);
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);

        while(range.getDistance(DistanceUnit.CM) > 12){
            //strafes LEFT
            double  val = .5;
            left1.setPower(val);
            left2.setPower(-val);
            right1.setPower(-val);
            right2.setPower(val);
            telemetry.addData("raw ultrasonic", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);
        telemetry.update();
        encPos = left1.getCurrentPosition();
        drive(.3, left1, right1, left2, right2);
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 300) {
            telemetry.addData("Left", left1.getCurrentPosition() - encPos);
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);
        encPos = left1.getCurrentPosition();
        while(Math.abs((left1.getCurrentPosition()-encPos)) < 150){
            //strafes LEFT
            double  val = -.2;
            left1.setPower(val);
            left2.setPower(-val);
            right1.setPower(-val);
            right2.setPower(val);
            telemetry.addData("raw ultrasonic", range.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        drive(0, left1, right1, left2, right2);

/*
        while(Math.abs(gyroSensor.getHeading()-startingG) > 5){
            double val = .2;
            left1.setPower(val);
            left2.setPower(-val);
            right1.setPower(-val);
            right2.setPower(val);
        }
        drive(0, left1, right1, left2, right2);
*/
        while( ((color1.red()+color1.blue()+color1.green())/3) < 2){
            drive(-.2, left1, right1, left2, right2);
        }
        drive(0, left1, right1, left2, right2);

        //AT FIRST BEACON
        if(color2.red() > 5){
            rightPush.setPosition(.5);
        } else {
            leftPush.setPosition(.5);
        }
        sleep(1000);

        drive(-.2, left1, right1, left2, right2);
        sleep(500);
        leftPush.setPosition(0);
        rightPush.setPosition(0);
        while( ((color1.red()+color1.blue()+color1.green())/3) < 2) {
        }
        drive(0, left1, right1, left2, right2);
        //AT SECOND BEACON
        if(color2.red() > 5){
            rightPush.setPosition(.5);
        } else {
            leftPush.setPosition(.5);
        }

    }

    //Constant Power
    public static void drive(double val, DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        left1.setPower(val);
        left2.setPower(val);
        right1.setPower(val);
        right2.setPower(val);
    }

    //Mecanum:
    // y - forwards
    // x - side
    // c - rotation
    public static void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }

    //resets encoders
    public static void resetEncs(DcMotor left1, DcMotor right1, DcMotor left2, DcMotor right2){
        while(left1.getCurrentPosition() != 0){
            left1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            left2.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right1.setMode(DcMotor.RunMode.RESET_ENCODERS);
            right2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
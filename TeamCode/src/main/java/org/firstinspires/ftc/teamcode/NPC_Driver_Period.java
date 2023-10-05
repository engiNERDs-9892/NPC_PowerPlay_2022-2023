package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
// if(jack.isDriving){
 //crash
 //}

@TeleOp(name="NPC Driver Period", group="Linear Opmode")

public class NPC_Driver_Period extends LinearOpMode {

    // The private DcMotor _____; are used to identify a motor that can be used throughout the code

    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor motorLS = null;
    private DcMotor motorLS2 = null;


    // The Servo ____; are used to identify a servo that can be used throughout the code.

    Servo servoL;
    Servo servoR;

    // The Touch Sensor.

    TouchSensor touch;
    TouchSensor toucht;
    @Override
    public void runOpMode() {

        // HardwareMap Section (Used to talk to the driver hub for the configuration)

        // Motors

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLS = hardwareMap.dcMotor.get("motorLS");
        motorLS2 = hardwareMap.dcMotor.get("motorLS2");

        // Servo

        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");

        // Sensor

        touch = hardwareMap.get(TouchSensor.class, "touch");
        toucht = hardwareMap.get(TouchSensor.class, "toucht");

        // This sets the motor power to zero, so the motors DO NOT run during the initialize phase

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        motorLS.setPower(0);
        motorLS2.setPower(0);


        // Just tells it to go in a certain direction
        // NOTE: You only have to do this for the wheels for the calculation down below
        // the rest doesn't matter because you can just add a negative to the power to invert it

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorLS.setDirection(DcMotor.Direction.FORWARD);
        motorLS2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Variable used for Regular speed (To find the direction that the stick needs to be in)
            double max;

            // Variable used for Fast speed (To find the direction that the stick needs to be in)
            double fmax;

            // Variable used for Slow speed (To find the direction that the stick needs to be in)
            double smax;


            double ls = gamepad2.left_stick_y; // Linear Slide Up and Down)


            double axial = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value

            // The code below talks about the X-axis (Left and Right)

            double lateral = -gamepad1.left_stick_x; // The bottom two are inverted because motor direction is changed

            // The code below talks about Z-Axis (Spinning around)

            double yaw = -gamepad1.right_stick_x;


            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Regular speed
            double leftFrontPower = .5 * (axial + lateral + yaw);
            double rightFrontPower = .5 * (axial - lateral - yaw);
            double leftBackPower = .5 * (axial - lateral + yaw);
            double rightBackPower = .5 * (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Fast speed
            double FleftFrontPower = (axial + lateral + yaw);
            double FrightFrontPower = (axial - lateral - yaw);
            double FleftBackPower = (axial - lateral + yaw);
            double FrightBackPower = (axial + lateral - yaw);

            // Combine the joystick requests for each axis-motion to determine each wheel's power
            // And direction for Slow speed
            double SleftFrontPower = .2 * (axial + lateral + yaw);
            double SrightFrontPower = .2 * (axial - lateral - yaw);
            double SleftBackPower = .2 * (axial - lateral + yaw);
            double SrightBackPower = .2 * (axial + lateral - yaw);


            ////////////////////////////////////////////////////////////////////////////////////////////
            // use LEFT joystick to go Forward/Backwards & left/Right, and RIGHT joystick to Rotate.///
            //////////////////////////////////////////////////////////////////////////////////////////


            // This calculates the direction & power for Regular Speed
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            // This calculates the direction & power for Fast Speed
            fmax = Math.max(Math.abs(FleftFrontPower), Math.abs(FrightFrontPower));
            fmax = Math.max(fmax, Math.abs(FleftBackPower));
            fmax = Math.max(fmax, Math.abs(SrightBackPower));

            // This calculates the direction & power for Fast Speed
            smax = Math.max(Math.abs(SleftFrontPower), Math.abs(SrightFrontPower));
            smax = Math.max(smax, Math.abs(SleftBackPower));
            smax = Math.max(smax, Math.abs(SrightBackPower));


            // sets the wheels to do whatever the calculation above tells it to do Fast Speed
            if (fmax > 1.0) {

                FleftFrontPower = fmax;
                FrightFrontPower = fmax;
                FleftBackPower = fmax;
                FrightBackPower = fmax;
            }

            // sets the wheels to do whatever the calculation above tells it to do for Slow Speed
            if (smax > 1.0) {
                SleftFrontPower = smax;
                SrightFrontPower = smax;
                SleftBackPower = smax;
                SrightBackPower = smax;
            }

            // sets the wheels to do whatever the calculation above tells it to do for Regular Speed
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            // Setting the power for Slow Speed
            if (gamepad1.left_trigger != 0) {
                motorFL.setPower(SleftFrontPower);
                motorBL.setPower(SleftBackPower);
                motorBR.setPower(SrightBackPower);
                motorFR.setPower(SrightFrontPower);

            }


            // Setting the power for Regular Speed
            else {
                motorFL.setPower(leftFrontPower);
                motorBL.setPower(leftBackPower);
                motorFR.setPower(rightFrontPower);
                motorBR.setPower(rightBackPower);
            }


            //////////////////////////////////////
            ////   Code for Linear Slide    //////
            //////////////////////////////////////

            // Code below says if you are doing nothing, don't move the Lead Screw

            if (ls == 0) {
                motorLS.setPower(0);
                motorLS2.setPower(0);
            }

            // Code below tells it to go up.

            if (ls < 0) {
                motorLS.setPower(1 * ls);
                motorLS2.setPower(1 * ls);
            }

            // Code below tells it to go down

            if (ls > 0) {
                if(toucht.isPressed()){
                    motorLS.setPower(0.25);
                    motorLS2.setPower(0.25);
                }
                if (touch.isPressed()) {
                    motorLS.setPower(0);
                    motorLS2.setPower(0);
                } else {
                    motorLS.setPower(0.5 * ls);
                    motorLS2.setPower(0.5 * ls);
                }
            }


            //////////////////////////////////////////////////////////////
            /////      Servo Code for arms left and right        /////////
            /////////////////////////////////////////////////////////////

            // Close arms

            if (gamepad2.a) {
                servoL.setPosition(0);
                servoR.setDirection(Servo.Direction.REVERSE);
                servoR.setPosition(0);
            }

            // Open arms

            if (gamepad2.y) {
                servoL.setPosition(.25);
                servoR.setDirection(Servo.Direction.REVERSE);
                servoR.setPosition(.25);
            }
//set height to mid
            if (gamepad2.dpad_down) {
                mid(500,-1);

            }
        }
    }

    private void mid (int time, double speed) {
        motorLS.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLS2.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLS.setPower(speed);
        motorLS2.setPower(speed);

        sleep(time);
        motorLS.setPower(0);
        motorLS2.setPower(0);
    }

    }
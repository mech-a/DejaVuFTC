/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.ARM_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.DRIVE_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.SENSOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.ConfigurationNames.SERVO_MOTOR_NAMES;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.MARKER_HELD;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_LOCKED;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_UNLOCKED;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.TELESCOPING_MAX_POSITION;


/**
 * POV Drive mode with encoder driving compatibility w/o while loops
 */

@TeleOp(name="POV Drive", group="Competition")
//@Disabled
public class POVDriveRobot extends LinearOpMode {

    // Declare OpMode members.
    Robot r = new Robot(this, Enums.OpModeType.TELEOP);



    double servoPosition = 1;


    int count = 0;
    int LIM = -300;



    double[] g1 = new double[4];
    double[] g2 = new double[4];
    double[] g1Adjusted = new double[4];
    double[] g2Adjusted = new double[4];

    double modifier = 0.25, speedSwitchSlow = 0.25, speedSwitchFast = 1, speedSwitchPow = 1;

    double powL = 0, powR = 0;
    
    double powIntake = 0, powIntakeMax = 0.5, powIntakeMin = -0.5;

    double powLift = 0, powLiftMax = 1, powLiftMin = -1;

    double powRotate = 0, powRotateOutwards = 0.5, powRotateTowardsRobot = -0.5;


    double powTelescope = 0;
    
    final double TRIGGER_DEADZONE = 0.3;

    boolean telescopingMax = false, telescopingMin = false;

    boolean runSlow = false, runFast = false, runExponential = false;

    //code spec:
    // powLift; 1 for raise, -1 for fall
    // powIntake; 1 for intake, -1 for expel
    // powRotate; 1 for outwards, -1 for inwards
    // powTelescope; 1 for extend, -1 for retract


    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        r.start(hardwareMap, telemetry);
        r.init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            setGamepads(modifier);

            adjustPowers();


            telemetry.addData("Rotation", r.armMotors[2].getCurrentPosition());

            //Button handling
            //L/R Trigger, intake
            intake();

            //U/D Dpad, lift(raise)
            raise();

            //X,B rotation
            rotation();

            //DPAD L, R telescoping
            telescope();

            speedSwitch();


            //G2 LB + RB - Servo
            jamServoControl();

            setPowers();

            telemetryStack();


            telemetry.update();


            sleep(100);






            //TODO see if vertical motor can be floated due to surgical tubing 
            



            //TODO a "bug" is that if the next tick the current position is instantaneously less than or equal to 0,
            // both telescopingMax and Min will be true, therefore the motor will lock.
            // I want to test to see if this case happens, which most likely will, then we can changed the code by making
            // two different if/else clauses. Let's just try it out!
            // also good thing for eng nb
            // or, we can keep this as is annd make the second powIntake handling part an else if so only 1 runs
//            if(r.armMotors[1].getCurrentPosition()>=(TELESCOPING_MAX_POSITION))
//                telescopingMax = true;
//            else if(r.armMotors[1].getCurrentPosition()<=0)
//                telescopingMin = true;
//            else {
//                telescopingMax = false;
//            }
//
//
//            //TODO check if hardware cycle will allow this to run correctly
//            if(telescopingMax) {
//                if(powTelescope > 0)
//                    powTelescope = 0;
//            }
//
//            if(telescopingMin) {
//                if(powTelescope < 0)
//                    powTelescope = 0;
//            }













        }
    }

    private void adjustPowers() {
        powL = Range.clip(g1[1] + g1[2], -1, 1);
        powR = Range.clip(g1[1] - g1[2], -1, 1);
    }

    private void intake() {
        if(gamepad2.right_trigger > TRIGGER_DEADZONE)
            powIntake = powIntakeMax;
        else if (gamepad2.left_trigger > TRIGGER_DEADZONE)
            powIntake = powIntakeMin;
        else
            powIntake = 0;
    }

    private void raise() {
        if(gamepad2.dpad_up)
            powLift = powLiftMax;
        else if (gamepad2.dpad_down)
            powLift = powLiftMin;
        else
            powLift = 0;
    }

    private void rotation() {
        if(gamepad2.b) {
            if(r.armMotors[2].getCurrentPosition() <= LIM) {
                powRotate = powRotateTowardsRobot/2;
                telemetry.addData("Rot", "in, past lim");
            }
            else {
                powRotate = powRotateTowardsRobot*2;
            }
        }
        else if(gamepad2.x) {
            if(r.armMotors[2].getCurrentPosition() >= LIM) {
                powRotate = powRotateOutwards/2;
                telemetry.addData("Rot", "out, past lim");
            }
            else {
                powRotate = powRotateOutwards*2;
            }
        }
        else {
            powRotate = 0;
        }


        telemetry.addData("Powrot", powRotate);
    }

    private void telescope() {
        if(gamepad2.left_bumper) {
            if(r.armMotors[1].getCurrentPosition() <=20) {
                powTelescope = 0;
                telemetry.addData("Err", "Horizontal pulled!");
            }
            powTelescope = 0.5;
        }
        else if(gamepad2.right_bumper) {
            powTelescope= -0.5;

        }
        else {
            powTelescope = 0;
        }
    }

    private void setPowers() {
        r.driveMotors[0].setPower(powL);
        r.driveMotors[1].setPower(powR);
        r.driveMotors[2].setPower(powR);
        r.driveMotors[3].setPower(powL);

        r.armMotors[0].setPower(powLift);
        r.armMotors[1].setPower(powTelescope);
        r.armMotors[2].setPower(powRotate);
        r.armMotors[3].setPower(powIntake);

        r.servoMotors[1].setPosition(servoPosition);
    }

    private void setGamepads(double modifier) {
        //left joystick x, y, right joystick x, y
        g1[0] = gamepad1.left_stick_x * modifier;
        g1[1] = -gamepad1.left_stick_y * modifier;
        g1[2] = gamepad1.right_stick_x * modifier;
        g1[3] = -gamepad1.right_stick_y * modifier;

        g2[0] = gamepad2.left_stick_x * modifier;
        g2[1] = -gamepad2.left_stick_y * modifier;
        g2[2] = gamepad2.right_stick_x * modifier;
        g2[3] = -gamepad2.right_stick_y * modifier;

        //TODO deadzones
    }


    private void speedSwitch() {
//        if (gamepad1.y) {
//            runExponential = !runExponential;
//        }


        if(gamepad1.left_bumper) {
            runSlow = true;
            runFast = false;
        }

        else if (gamepad1.right_bumper) {
            runSlow = false;
            runFast = true;
        }

        if(runFast) {
            speedSwitchPow = speedSwitchFast;
        }

        else if(runSlow) {
            speedSwitchPow = speedSwitchSlow;
        }

//        if(runExponential) {
//            g1[1] =  - (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * modifier;
//            g1[2] =  (gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x)) * modifier;
//
//            adjustPowers();
//
//            telemetry.addData("SpeedType:", "Exponential");
//        }

        powL = speedSwitchPow * powL;
        powR = speedSwitchPow * powR;

    }

    private void jamServoControl() {
        if(gamepad2.y)
            servoPosition = SERVO_LOCKED;
        else if(gamepad2.a)
            servoPosition = SERVO_UNLOCKED;


        //start (zero) will be at 1

    }

    private void telemetryStack() {

        telemetry.addData("Drive Powers", "L (%3f) : R (%3f)", powL, powR);
        telemetry.addData("Servo Pos", "Marker (%3f) : Locker (%3f)",
                r.servoMotors[0].getPosition(), r.servoMotors[1].getPosition());
        telemetry.addData("Speed Mod:", speedSwitchPow * modifier);

//        if(runExponential) {
//            telemetry.addData("Speed Type", "Exponential");
//        }
//        else {
//            telemetry.addData("Speed Type", "Linear");
//        }

    }




    private void autonRotation() {


    }




//    private void servoMotorsInit(){
//        for(int i = 0; i<2; i++){
//            r.servoMotors[i] = hardwareMap.servo.get(SERVO_MOTOR_NAMES[i]);
//        }
//
//
//        //no need to change servos for teleop
//
//    }
//
//    private void driveMotorsInit() {
//        for (int i = 0; i<4; i++) {
//            r.driveMotors[i] = hardwareMap.dcMotor.get(DRIVE_MOTOR_NAMES[i]);
//
//            //TODO standardize between robot and code the port numbers and i
//            /*
//            switch (i) {
//                case 0: driveMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                    break;
//                case 1: driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
//                    break;
//                case 2: driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
//                    break;
//                case 3: driveMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                    break;
//
//            }*/
//
//            if(i%3==0)
//                r.driveMotors[i].setDirection(DcMotor.Direction.REVERSE);
//            else
//                r.driveMotors[i].setDirection(DcMotor.Direction.FORWARD);
//
//            r.driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            r.driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            r.driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    private void armMotorsInit() {
//        for (int i = 0; i<4; i++) {
//            r.armMotors[i] = hardwareMap.dcMotor.get(ARM_MOTOR_NAMES[i]);
//
//            //TODO change if needed: well, i did it, but must be changed for when telescoping works
//            /*
//            switch (i) {
//                case 0: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                        break;
//                case 1: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                    break;
//                case 2: armMotors[i].setDirection(DcMotor.Direction.FORWARD);
//                    break;
//                case 3: armMotors[i].setDirection(DcMotor.Direction.REVERSE);
//                    break;
//            }*/
//
//            if(i==3)
//                armMotors[i].setDirection(DcMotor.Direction.REVERSE);
//            else
//                armMotors[i].setDirection(DcMotor.Direction.FORWARD);
//
//            if(!caller.isStopRequested()) {
//                armMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                armMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                armMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//    }
//
//    private void imuInit() {
//        imu = hardwareMap.get(BNO055IMU.class, SENSOR_NAMES[0]);
//        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();
//        gyroParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        gyroParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        gyroParameters.loggingEnabled      = true;
//        gyroParameters.loggingTag          = "IMU";
//        if(!caller.isStopRequested()) {
//            imu.initialize(gyroParameters);
//            //TODO using angles while opmode is not active could cause problems
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        }
//
//    }


}

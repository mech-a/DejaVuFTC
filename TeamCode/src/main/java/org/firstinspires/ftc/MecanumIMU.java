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

package org.firstinspires.ftc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.dependencies.Enums;
import org.firstinspires.ftc.teamcode.dependencies.Enums.*;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import static org.firstinspires.ftc.teamcode.dependencies.Enums.ArmMotor.INTAKE;
import static org.firstinspires.ftc.teamcode.dependencies.Enums.ArmMotor.RAISE;
import static org.firstinspires.ftc.teamcode.dependencies.Enums.ArmMotor.ROTATE;
import static org.firstinspires.ftc.teamcode.dependencies.Enums.ArmMotor.TELESCOPE;


/**
 * MecanumIMU
 * Read more about the code on https://www.chiefdelphi.com/media/papers/2390
 */

@TeleOp(name="MecanumIMU", group="#")
//@Disabled
public class MecanumIMU extends LinearOpMode {

    // Declare OpMode members.
    Robot r = new Robot(this, OpModeType.TELEOP);

    double[] g1 = new double[4];

    //g2 f(x):
    /*

    dpad U/D raise
    rt/lt in/ex


    */

    // 1/8 max
    double modifier = 0.25, speedSwitchSlow = 0.25, speedSwitchFast = 1, speedSwitchPow = 1;
    static double DEADZONE = 0.15, TRIGGER_DEADZONE = 0.3;

    boolean runSlow = false, runFast = false;

    double powIntakeMax, powIntakeMin;
    
    // do tuning of lt/rt & see if counts can change

    double powFL, powFR, powBR, powBL;

    double[] driveMotorPows = new double[4];

    //Raise,Telescope,Rotate,Intake
    //Min
    //Max

    //todo make into enum or constant
    double[][] armMotorPows = {
            {0,0,0,0},
            {-1,-0.75,-0.5,-0.6},
            {1,0.75,0.5,1}
    };

    double intakepower = 0;

    double servoPosition;

    double fwd, strafe, rotate;

    public enum DriveMode {
        FIELD, CARTESIAN
    }

    // automatically set the drivemode to field
    DriveMode driveMode = DriveMode.FIELD;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        r.start(hardwareMap,telemetry);
        r.init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        // TODO create teleop parent class that extends linearopmode w gamepad f(x)s

        while (opModeIsActive()) {
            // basic mecanum driving that is respective to the robot
            setGamepads();

            speedSwitch();

            mecanum();

            jamServoControl();

            if(gamepad1.y) {
                driveMode = DriveMode.CARTESIAN;
            }
            else if (gamepad1.a) {
                driveMode = DriveMode.FIELD;
            }

            if(gamepad1.dpad_up) {
                //todo figure out reset angle
                r.imuInit();
            }
            
            armMotors();

            telemetry.addData("Mode", driveMode.toString());



            setPowers();

            telemetry.update();

            sleep(50);
        }
    }
    
    private void armMotors() {
        raise();
        telescope();
        rotate();
        intake();
    }
    
    //TODO auto buttons
    //TODO make this function able to take in an object of 3 different vals (dcmotor, upper, lower) and does this exact thing : )
    private void raise() {
        boundedMotor(gamepad2.dpad_up, gamepad2.dpad_down, RAISE.getIndex(), 0,-730);
    }

    private void telescope() {
        boundedMotor(gamepad2.right_bumper, gamepad2.left_bumper, TELESCOPE.getIndex(), 0, -879);
    }

    private void rotate() {
        boundedMotor(gamepad2.b, gamepad2.x, ROTATE.getIndex(), 0, -431);
    }

    private void intake() {
        telemetry.addData("RT:", gamepad2.right_trigger);
        telemetry.addData("LT:", gamepad2.left_trigger);
        if(gamepad2.right_trigger > TRIGGER_DEADZONE) {
            telemetry.addData("intake:", "inwards");
            telemetry.update();
            //intakepower = -0.5;
            armMotorPows[0][INTAKE.getIndex()] = armMotorPows[2][INTAKE.getIndex()];
        } else if(gamepad2.left_trigger > TRIGGER_DEADZONE) {
            armMotorPows[0][INTAKE.getIndex()] = armMotorPows[1][INTAKE.getIndex()];
            telemetry.addData("intake:", "outwards");
            telemetry.update();
            //intakepower = 0.5;
        } else if(gamepad2.right_trigger < TRIGGER_DEADZONE && gamepad2.left_trigger < TRIGGER_DEADZONE) {
            armMotorPows[0][INTAKE.getIndex()] = 0;
            //intakepower = 0;
        }
    }

    //make each motor enum'd with two parts; the motor and an identifier, so we can reference the correct
    //TODO make telem for bounds
    private void boundedMotor(boolean out, boolean in, int motorNum, double upperBound, double lowerBound) {
        if(out
                && r.armMotors[motorNum].getCurrentPosition() < upperBound)
            armMotorPows[0][motorNum] = armMotorPows[2][motorNum];
        else if (in
                && r.armMotors[motorNum].getCurrentPosition() > lowerBound)
            armMotorPows[0][motorNum] = armMotorPows[1][motorNum];
        else
            armMotorPows[0][motorNum] = 0;
    }

    private void mecanum() {
        if(driveMode == DriveMode.FIELD) {
            double heading = r.getHeading();
            telemetry.addData("Heading", heading);

            heading = Math.toRadians(heading);


            if(heading > 0) {
                //ccw
                fwd = g1[1] * Math.cos(Math.abs(heading)) - g1[0] * Math.sin(Math.abs(heading));
                strafe = g1[1] * Math.sin(Math.abs(heading)) + g1[0] * Math.cos(Math.abs(heading));
            }
            else {
                //cw
                fwd = g1[1] * Math.cos(Math.abs(heading)) + g1[0] * Math.sin(Math.abs(heading));
                strafe = -g1[1] * Math.sin(Math.abs(heading)) + g1[0] * Math.cos(Math.abs(heading));
            }

            rotate = g1[2];

            powFL = fwd + rotate + strafe;
            powFR = fwd - rotate - strafe;
            powBL = fwd + rotate - strafe;
            powBR = fwd - rotate + strafe;

        }

//        else if (driveMode == DriveMode.POLAR) {
//            double radius = Math.hypot(g1[1], g1[0]);
//            double angle = Math.atan2(g1[1], g1[0]) - Math.PI / 4;
//            powFL = radius * Math.cos(angle) + g1[2];
//            powFR = radius * Math.sin(angle) - g1[2];
//            powBL = radius * Math.cos(angle) + g1[2];
//            powBR = radius * Math.sin(angle) - g1[2];
//
//        }

        else {

            powFL = g1[1] + g1[2] + g1[0];
            powFR = g1[1] - g1[2] - g1[0];
            powBL = g1[1] + g1[2] - g1[0];
            powBR = g1[1] - g1[2] + g1[0];

        }
    }
    private void jamServoControl() {
        if(gamepad2.y)
            servoPosition = 0;
        else if(gamepad2.a)
            servoPosition = 0.5;


        //start (zero) will be at 1

    }

    private void speedSwitch() {

        if (gamepad1.left_trigger > TRIGGER_DEADZONE) {
            runSlow = true;
            runFast = false;
        } else if (gamepad1.right_trigger > TRIGGER_DEADZONE) {
            runSlow = false;
            runFast = true;
        }

        if (runFast) {
            speedSwitchPow = speedSwitchFast;
        } else if (runSlow) {
            speedSwitchPow = speedSwitchSlow;
        }
    }

        private void setGamepads () {

            //left joystick x, y, right joystick x, y
            g1[0] = gamepad1.left_stick_x;
            g1[1] = -gamepad1.left_stick_y;
            g1[2] = gamepad1.right_stick_x;
            g1[3] = -gamepad1.right_stick_y;

            for (int i = 0; i < g1.length; i++)
                g1[i] = (Math.abs(g1[i]) > DEADZONE ? g1[i] : 0) * modifier;


        }

        private void setPowers () {
            r.driveMotors[0].setPower(powFL);
            r.driveMotors[1].setPower(powFR);
            r.driveMotors[2].setPower(powBR);
            r.driveMotors[3].setPower(powBL);

            // r.armMotors[3].setPower(intakepower);

            for (int i = 0; i <= armMotorPows.length; i++) {
                r.armMotors[i].setPower(armMotorPows[0][i]);
            }

//        r.armMotors[0].setPower(powRaise);
//        r.armMotors[1].setPower(powTelescope);
//        r.armMotors[2].setPower(powRotate);
//        r.armMotors[3].setPower(powIntake);

            r.servoMotors[1].setPosition(servoPosition);
        }

}
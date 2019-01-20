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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.dependencies.Robot;
import org.opencv.core.Mat;


/**
 * Ramping Speed
 * CB: Gaurav
 */
//@Disabled
@TeleOp(name="Ramping Speed", group="Internal")

public class RampingSpeed extends LinearOpMode {

    // Declare OpMode members.
    private double power = 0;
    private double epsilon = 0.1;
    private double numSteps = 10;
    private double numStepsMin = numSteps/=2;
    private double numStepsMax = numSteps;
    private double count = 0;
    private double joystick;
    private double oldJoystick;
    Robot r = new Robot(this);


    double modifier = 0.5;

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        //TODO combine r.start and r.init
        r.start(hardwareMap, telemetry);
        r.init();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            joystick = -gamepad1.left_stick_y * modifier;


            //Every five repetitions of the drive loop, the oldJoystick value updates
            if(count % 5 == 0)
                oldJoystick = joystick;

            //If the robot is decelerating, we want it to ramp faster than if it is accelerating
            if(Math.abs(oldJoystick)< Math.abs(joystick)) {
                numSteps = numStepsMin;
            }
            else {
                numSteps = numStepsMax;
            }

            //If the difference between the motor power and joystick value is negligible, the driver
            //is trying to change the speed of the robot, so the code limits the rate at which the
            //motor power can be changed
            if(Math.abs(joystick-power)>epsilon)
                power += (joystick-power)/numSteps;

            //If the joystick values are non-negligible, set the motor power to 0
            if(Math.abs(oldJoystick) < epsilon && Math.abs(joystick) < epsilon)
                power = 0;

            r.driveMotors[0].setPower(power);
            r.driveMotors[1].setPower(power);
            r.driveMotors[2].setPower(power);
            r.driveMotors[3].setPower(power);

            sleep(50);

            telemetry.addData("Joystick", joystick);
            telemetry.addData("power", power);
            telemetry.update();

            count++;
        }
    }
}

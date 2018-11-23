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
 * promote products derived from this software without specific prior written perm!isSion.
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

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.dependencies.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.HD_COUNTS_PER_INCH;


@Autonomous(name = "Turn 90", group = "Sensor")
//@Disabled
public class Turn90 extends LinearOpMode {
    Robot r = new Robot(this);
    private int driveMtrTarget;
    private int angle = 45;
    private double speed = 0.1;
    int a,b;
    String direction = "ccw";

    @Override public void runOpMode() {
        r.start(hardwareMap, telemetry);
        r.init();

        // Wait until we're told to go
        telemetry.addData("Hz", r.getGyroHertz().toString());
        telemetry.update();

        waitForStart();

        //todo direction enums


        //todo rotate by counts


        //while(opModeIsActive()) {
            if(direction.equals("ccw")) {
                a = -1;
                b = 1;
                direction = "c";
            }
            else {
                a = 1;
                b = -1;
                direction = "ccw";
            }


            //counts per degree
            //avg(723,733,728,737)/85.6875

            double countsPerDegree = ((723+733+728+737)/4.0)/85.6875;

            driveMtrTarget = (int) (angle * countsPerDegree);

            for (int i = 0; i<4 && !isStopRequested(); i++) {
                if(i % 3 == 0) {
                    r.driveMotors[i].setTargetPosition(driveMtrTarget * a);
                }
                else {
                    r.driveMotors[i].setTargetPosition(driveMtrTarget * b);
                }

                r.driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                r.driveMotors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            telemetry.addData("S", "Run to position complete");
            telemetry.update();
            sleep(1000);

            // try this out
            // caller.sleep(750);

            for (int i = 0; i<4 && !isStopRequested(); i++) {
                r.driveMotors[i].setPower(speed);
            }

            telemetry.addData("S", "speed set");
            telemetry.update();
            sleep(1000);

            while(!isStopRequested() &&
                    ((r.driveMotors[0].isBusy()) && (r.driveMotors[1].isBusy()) && (r.driveMotors[2].isBusy()) && (r.driveMotors[3].isBusy())) ) {
                //TODO change telemetry name to enum
                telemetry.addData("0mtrFl", "%7d : %7d",
                        r.driveMotors[0].getCurrentPosition(), driveMtrTarget);
                telemetry.addData("1mtrFR", "%7d : %7d",
                        r.driveMotors[1].getCurrentPosition(), driveMtrTarget);
                telemetry.addData("2mtrBR", "%7d : %7d",
                        r.driveMotors[2].getCurrentPosition(), driveMtrTarget);
                telemetry.addData("3mtrBL", "%7d : %7d",
                        r.driveMotors[3].getCurrentPosition(), driveMtrTarget);

                telemetry.update();
            }

            for (int i = 0; i<4 && !isStopRequested(); i++) {
                r.driveMotors[i].setPower(0);
            }
            for (int i = 0; i<4 && !isStopRequested(); i++) {
                r.driveMotors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("heading", "%7f : %7f", angle, r.getHeading());
            telemetry.update();
            sleep(1000);

            //dont let it run past 180!
            angle+=5;


        //}
        

    }
}

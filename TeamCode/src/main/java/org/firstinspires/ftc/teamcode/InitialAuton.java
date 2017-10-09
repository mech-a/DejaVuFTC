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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Initial Autonomous code that implements
 *  -an encoder drive
 *  -mecanum mobility
 *  -various functions called from an object of another class
 *  -vuforia implementation
 *  -BNO055IMU gyroscopic capability
 *
 *
 * License Key
 * AboeStD/////AAAAGaA8AMxAS0isjCVUhD5lHuRymY1yqEVbDu1/PRTIEg/9JzZxKpV/P39rY/QC64WcjeCtnUDq0jj7yWEPkWZClLRC2KVwsjQPUe/mjwl6y51KfIKgSulpN63fEYOBdY5ZR4fNswicR46PElRn4NaKqkV6fr9cLS62V8Oa8ow88oUK3xga8OJkNYf+3VoIQ7dj/RxiKzQCBJRt2ZbRIUimTFw4oTC5LJ/NXV2jSD+m7KnW7TCpC7n/7hRxyKRmw+JKGoz5kJIfxhliqs1XD3MnD9KN5w6cEwEmg3uYUZ5Bx7bcuON/uEaqifBnmwpdI0Vjklr67kMVYb27z1NsC+OB7moGIPdjhKho6nhwLy9XyMPw
 *  Initially created by gb
 */

@Autonomous(name="OpModeStart", group="Testing Autons")
@Disabled
public class InitialAuton extends LinearOpMode {

    // Declare OpMode members.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {


        }
    }

    public void movement(String direction, double power, double distInCm) {

    }
}

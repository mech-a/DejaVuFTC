<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> parent of 4a1ee26... Changes
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
=======
>>>>>>> parent of 34196b3... i m gabi lol
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.dependencies.Robot;

import static org.firstinspires.ftc.teamcode.dependencies.Constants.HD_COUNTS_PER_REV;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.SERVO_UNLOCKED;
import static org.firstinspires.ftc.teamcode.dependencies.Constants.TELESCOPING_MAX_POSITION;


/**
 * Copy Me Linear
 */

@TeleOp(name="Tilting arm test", group="Internal")
@Disabled
public class Armtesting extends LinearOpMode {

    Robot r = new Robot(this);
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // run until the end of the match (driver presses STOP)
        r.positionDrive(2, (int)HD_COUNTS_PER_REV/4, 0.25);
    }
}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
//-260
=======
>>>>>>> parent of 4a1ee26... Changes
=======
//-260
>>>>>>> parent of 34196b3... i m gabi lol
=======
//-260
>>>>>>> parent of 34196b3... i m gabi lol
=======
//-260
>>>>>>> parent of 34196b3... i m gabi lol

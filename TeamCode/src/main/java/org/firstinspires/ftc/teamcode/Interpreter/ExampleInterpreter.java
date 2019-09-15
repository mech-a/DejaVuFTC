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

package org.firstinspires.ftc.teamcode.Interpreter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.dependencies.Robot;

import java.io.IOException;
import java.lang.reflect.Method;


/**
 * Copy Me Linear
 */

@Autonomous(name="ExampleInterpreter", group="Internal")
@Disabled
public class ExampleInterpreter extends DejaVuLinearOpMode {

    // Declare OpMode members.
    String fname = "ExampleInterpreter.json";

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)

        Robot r = new Robot(this);

        download(fname);

        try {
            convertJSONToBOJO();
        } catch (IOException e) {
            //TODO log.d
            e.printStackTrace();
        }

        waitForStart();

        // run until the end of the match (driver presses STOP)
        //TODO implement reflection
        for(MethodData data : auton.methods) {
            try{
                if(data.functionParams != null) {
                    Method method = data.getClassObj().getMethod(data.functionName, data.getParamTypes());
                    //Method method = ExampleRobot.class.getMethod(data.functionName, data.getParamTypes());
                    method.invoke(r, data.functionParams);
                }
                else {
                    Method method = data.getClassObj().getMethod(data.functionName);
                    //Method method = ExampleRobot.class.getMethod(data.functionName);
                    method.invoke(r);
                }
            }
            catch(Exception e){
                e.printStackTrace();
            }
        }
    }
}

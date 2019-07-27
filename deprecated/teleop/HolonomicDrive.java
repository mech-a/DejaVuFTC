package org.firstinspires.ftc.teamcode.deprecated.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.deprecated.called.HWRobot;

import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.DEADZONE;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.NV60_SPEED;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SEVEN_INCHES_NV60;
import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SIXPOINTFIVE_NV60;
//import static org.firstinspires.ftc.teamcode.deprecated.called.RobotValues.SIX_INCHES_NV60;

/**
 * Created by gbhat on 8/20/2017.
 */

@TeleOp(name="Holonomic Drive", group="DriveOPs")
//@Disabled
public class HolonomicDrive extends LinearOpMode{
    HWRobot robot = new HWRobot();

    private double powFL = 0;
    private double powFR = 0;
    private double powBL = 0;
    private double powBR = 0;
    private double ch1,ch2,ch3,ch4;
    private double g2ch1, g2ch2, g2ch3, g2ch4;
    //double spdLinearUp = 1, spdLinearDown = -spdLinearUp;
    private boolean runSlow = false;
    private boolean runFast = false;
    private boolean linearRun = false;
    private boolean linUp = false;
    private boolean linDown = false;
    private double slowLimit = 0.25;
    private double fastLimit = 0.75;
    private double modifierValueDefault = 0.5;
    private double modifierValue = modifierValueDefault;
    //private double modifierValue = 1;


    double epsilon = 0.01;
    double totalDifference = 0;


    double powClawL = 0;
    double powClawR = 0;
    double clawSpeed = 0.5;
    private int slideLevel = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.getOpModeData(telemetry,hardwareMap);
        robot.init("motors");
        //robot.init("servos");
        prompt(telemetry, "Init", "HW initialized");

        waitForStart();

        while(opModeIsActive()) {
            setChannels();
            setPowers();
            setClawSpeeds();
            setClawPowers();

            //TODO make better solution for speed switching
            speedSwitch();

            linearSlideButtons();
            linearSlideControlOld();

            /*
            if(gamepad1.dpad_down) {
                robot.mtrLinear.setPower(-NV60_SPEED);
            }
            else if (gamepad1.dpad_up) {
                robot.mtrLinear.setPower(NV60_SPEED);
            }
            else {
                robot.mtrLinear.setPower(0);
            }*/

            modifyDriveValues();

            telemetry.addData("modifier value", modifierValue);

            telemetry.addData("MTRLin Pos:", robot.mtrLinear.getCurrentPosition());

            setDriveMotorPowers();
            sleep(50);
            telemetry.update();
        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }

    //Sets Gamepad info to channels
    private void setChannels() {
        ch1 = gamepad1.left_stick_x;
        ch2 = -gamepad1.left_stick_y;
        ch3 = gamepad1.right_stick_x;
        ch4 = -gamepad1.right_stick_y;

        g2ch1 = gamepad1.left_stick_x;
        g2ch2 = -gamepad1.left_stick_y;
        g2ch3 = gamepad1.right_stick_x;
        g2ch4 = -gamepad1.right_stick_y;

        if(Math.abs(ch1) <= DEADZONE) {
            ch1 = 0;
        }
        if(Math.abs(ch2) <= DEADZONE) {
            ch2 = 0;
        }
        if(Math.abs(ch3) <= DEADZONE) {
            ch3 = 0;
        }
        if(Math.abs(ch4) <= DEADZONE) {
            ch4 = 0;
        }


    }

    //Sets how channels determine motor powers (mecanum in this case)
    private void setPowers() {
        //correct
//        powFL = ch2 + ch1 + ch3;
//        powFR = ch2 - ch1 - ch3;
//        powBL = ch2 - ch1 + ch3;
//        powBR = ch2 + ch1 - ch3;

        powFL = ch1 + ch2 + ch3;
        powFR = ch1 - ch2 - ch3;
        powBL = ch1 - ch2 + ch3;
        powBR = ch1 + ch2 - ch3;


        //because wheel setup is wrong, try this; currently left on stick is right .
        //powFL = ch2 - ch1 + ch3;
        //powFR = ch2 + ch1 - ch3;
        //powBL = ch2 + ch1 + ch3;
        //powBR = ch2 - ch1 - ch3;

    }

    //Toggles between modifier values that affect how fast or slow the robot runs
    private void speedSwitch() {
        if(gamepad1.left_bumper) {
            runSlow = true;
            sleep(100);
        }
        else if(gamepad1.right_bumper) {
            runFast = true;
            sleep(100);
        }


        if(runSlow) {
            runFast = false;
            runSlow = false;
            if(modifierValue == modifierValueDefault) {
                modifierValue = slowLimit;
            }
            else if(modifierValue == slowLimit || modifierValue == fastLimit) {
                modifierValue = modifierValueDefault;
            }
            sleep(25);
        }

        else if(runFast) {
            runFast = false;
            runSlow = false;
            if(modifierValue == modifierValueDefault) {
                modifierValue = fastLimit;
            }
            else if(modifierValue == slowLimit || modifierValue == fastLimit) {
                modifierValue = modifierValueDefault;
            }
            sleep(25);
        }

    }
    
    private void linearSlideButtons(){
        //we want to have a couple different things in the linear slide;
        //we want manual control, automatic control that can go up and down
        //we use the booleans below to indicate what mode the slide should be doing

        if(gamepad1.y) {
            linearRun = true;
            linUp = true;
            linDown = false;
        }
        if(gamepad1.a) {
            linearRun = true;
            linUp = false;
            linDown = true;
        }
        //this is a force manual
        if(gamepad1.right_stick_button) {
            robot.mtrLinear.setPower(0);
            linearRun = false;
            linDown = false;
            linUp = false;
        }
    }
    private void linearSlideControlOld(){
        //the slide only needs to go up 12 inches but this means 3 levels of the slide
        //1) at ground 2) 6 inches up 3) 12 inches up
        //we want this system to be smart - we want to go to every 6 inch increment
        //we use slidelevel to signify which counts we want to go to
        if(linearRun) {
            if(gamepad1.dpad_up || gamepad1.dpad_down) {
                linearRun = false;
                linDown = false;
                linUp = false;
            }

            if(linUp) {
                if(robot.mtrLinear.getCurrentPosition() >= SIXPOINTFIVE_NV60) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                }
                else {
                    robot.mtrLinear.setPower(NV60_SPEED);
                }
            }
            else if(linDown) {
                if(robot.mtrLinear.getCurrentPosition() <= 100) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                }
                else {
                    robot.mtrLinear.setPower(-NV60_SPEED);
                }
            }

        }

        if(!linearRun) {
            if(gamepad1.dpad_up && robot.mtrLinear.getCurrentPosition() <= 2940) {
                robot.mtrLinear.setPower(NV60_SPEED);
            }
            else if(gamepad1.dpad_down && robot.mtrLinear.getCurrentPosition() >= 300) {
                robot.mtrLinear.setPower(-NV60_SPEED);
            }
            else {
                robot.mtrLinear.setPower(0);
            }
        }
    }


    private void linearSlideControl(){
        //the slide only needs to go up 12 inches but this means 3 levels of the slide
        //1) at ground 2) 6 inches up 3) 12 inches up
        //we want this system to be smart - we want to go to every 6 inch increment
        //we use slidelevel to signify which counts we want to go to
        if(linearRun) {
            if(linUp) {
                if(robot.mtrLinear.getCurrentPosition() >= slideLevel * SEVEN_INCHES_NV60) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                    if(slideLevel<2) {
                        slideLevel++;
                    }
                }
                else {
                    robot.mtrLinear.setPower(NV60_SPEED);
                }
            }
            else if(linDown) {
                if(robot.mtrLinear.getCurrentPosition() <= (slideLevel - 1) * SEVEN_INCHES_NV60 ) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                    if(slideLevel > 1) {
                        slideLevel--;
                    }
                }
                else {
                    robot.mtrLinear.setPower(-NV60_SPEED);
                }
            }

            if(gamepad1.dpad_up || gamepad1.dpad_down) {
                linearRun = false;
                linDown = false;
                linUp = false;
            }

        }

        if(!linearRun) {
            if(gamepad1.dpad_up && robot.mtrLinear.getCurrentPosition() <= SEVEN_INCHES_NV60 * 1.25) {
                robot.mtrLinear.setPower(NV60_SPEED);
            }
            else if(gamepad1.dpad_down && robot.mtrLinear.getCurrentPosition() >= 0) {
                robot.mtrLinear.setPower(-NV60_SPEED);
            }
            else {
                robot.mtrLinear.setPower(0);
            }
        }
    }

    //sets 2 buttons to extrude or intake glyphs
    private void setClawSpeeds() {
        //Adjusting speed of claw
        if(gamepad1.left_bumper) {
            totalDifference-=epsilon;
            totalDifference = Range.clip(totalDifference, -0.5, 0.5);
            sleep(20);
        }
        else if(gamepad1.right_bumper) {
            totalDifference+=epsilon;
            totalDifference = Range.clip(totalDifference, -0.5, 0.5);
            sleep(20);
        }
        else if(gamepad1.left_stick_button) {
            totalDifference = 0;
        }



        if(gamepad1.x) {
            powClawL = clawSpeed + totalDifference;
            powClawR = clawSpeed + totalDifference;
        }
        else if (gamepad1.b) {
            powClawL = -(clawSpeed + totalDifference);
            powClawR = -(clawSpeed + totalDifference);
        }
        else {
            powClawL = 0;
            powClawR = 0;
        }

        telemetry.addData("Claw Speed L:", powClawL);
        telemetry.addData("Claw Speed R:", powClawR);

    }

    //sets powers going to claws
    private void setClawPowers() {
        robot.mtrClawL.setPower(powClawL);
        robot.mtrClawR.setPower(powClawR);
    }

    /*
    private void resets() {
        if(gamepad1.dpad_up) {
            robot.resets("reverse");
        }
        else if (gamepad1.dpad_left || gamepad1.dpad_right) {
            robot.resets("normal");
        }
        else if (gamepad1.dpad_down) {
            robot.resets("stop");
        }
    }*/

    //modifies drive values
    private void modifyDriveValues() {
        powFL *= modifierValue;
        powFR *= modifierValue;
        powBL *= modifierValue;
        powBR *= modifierValue;
    }

    //sets final power going to motor
    private void setDriveMotorPowers(){
        robot.mtrFL.setPower(powFL);
        robot.mtrFR.setPower(powFR);
        robot.mtrBL.setPower(powBL);
        robot.mtrBR.setPower(powBR);
    }
}


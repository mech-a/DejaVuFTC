package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.called.HWRobot;

import static com.sun.tools.doclint.Entity.le;
import static org.firstinspires.ftc.teamcode.called.RobotValues.NV60_SPEED;
import static org.firstinspires.ftc.teamcode.called.RobotValues.SIX_INCHES_NV60;

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
    double powClawL = 0;
    double powClawR = 0;
    double clawSpeed = 0.5;
    private int slideLevel = 1;

    @Override
    public void runOpMode() {
        robot.getOpModeData(telemetry,hardwareMap);
        robot.init("motors");
        robot.init("servos");
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
            linearSlideControl();
            
            modifyDriveValues();

            telemetry.addData("modifier value", modifierValue);

            telemetry.addData("MTRLin Pos:", robot.mtrLinear);

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
        ch4 = -gamepad1.left_stick_y;

        g2ch1 = gamepad2.left_stick_x;
        g2ch2 = -gamepad1.left_stick_y;
        g2ch3 = gamepad1.right_stick_x;
        g2ch4 = -gamepad1.left_stick_y;
    }

    //Sets how channels determine motor powers (mecanum in this case)
    private void setPowers() {
        powFL = ch2 + ch1 + ch3;
        powFR = ch2 - ch1 - ch3;
        powBL = ch2 - ch1 + ch3;
        powBR = ch2 + ch1 - ch3;

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
            sleep(250);
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
            sleep(250);
        }

    }
    
    private void linearSlideButtons(){
        if(gamepad2.y) {
            linearRun = true;
            linUp = true;
            linDown = false;
        }
        if(gamepad2.a) {
            linearRun = true;
            linDown = true;
            linUp = false;
        }
        if(gamepad2.left_stick_button) {
            robot.mtrLinear.setPower(0);
            linearRun = false;
            linDown = false;
            linUp = false;
        }
    }
    
    private void linearSlideControl(){
        if(linearRun) {
            if(linUp) {
                robot.mtrLinear.setPower(NV60_SPEED);
                telemetry.update();
                if(robot.mtrLinear.getCurrentPosition() >= slideLevel * SIX_INCHES_NV60) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                    if(slideLevel<3) {
                        slideLevel++;
                    }
                }
            }
            else if(linDown) {
                robot.mtrLinear.setPower(-NV60_SPEED);
                if(robot.mtrLinear.getCurrentPosition() <= (slideLevel - 1) * SIX_INCHES_NV60 ) {
                    robot.mtrLinear.setPower(0);
                    linearRun = false;
                    linDown = false;
                    linUp = false;
                    if(slideLevel > 1) {
                        slideLevel--;
                    }
                }
            }

            if(gamepad2.dpad_up || gamepad2.dpad_down) {
                linearRun = false;
                linDown = false;
                linUp = false;
            }

        }

        if(!linearRun) {
            if(gamepad2.dpad_up) {
                robot.mtrLinear.setPower(NV60_SPEED);
            }
            else if(gamepad2.dpad_down) {
                robot.mtrLinear.setPower(-NV60_SPEED);
            }
            else {
                robot.mtrLinear.setPower(0);
            }
        }
    }

    //sets 2 buttons to extrude or intake glyphs
    private void setClawSpeeds() {
        if(gamepad2.x) {
            powClawL = clawSpeed;
            powClawR = clawSpeed;
        }
        else if (gamepad2.b) {
            powClawL = -clawSpeed;
            powClawR = -clawSpeed;
        }
        else {
            powClawL = 0;
            powClawR = 0;
        }
    }

    //sets powers going to claws
    private void setClawPowers() {
        robot.mtrClawL.setPower(powClawL);
        robot.mtrClawR.setPower(powClawR);
    }

    private void resets() {
        if(gamepad2.dpad_up) {
            robot.resets("reverse");
        }
        else if (gamepad2.dpad_left || gamepad2.dpad_right) {
            robot.resets("normal");
        }
        else if (gamepad2.dpad_down) {
            robot.resets("stop");
        }
    }

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


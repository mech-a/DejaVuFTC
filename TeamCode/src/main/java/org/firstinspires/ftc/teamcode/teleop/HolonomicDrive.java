package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.called.HWRobot;

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
    double spdLinearUp = 1, spdLinearDown = -spdLinearUp;
    double clawMid = 0.5;
    double clawIncrement = 0.01;
    double clawPos;
    private boolean runSlow = false;
    private boolean runFast = false;
    private boolean linearRun = false;
    private boolean linUp = false;
    private boolean linDown = false;
    private double slowLimit = 0.25;
    private double fastLimit = 0.75;
    private double modifierValueDefault = 0.5;
    private double modifierValue = modifierValueDefault;

    @Override
    public void runOpMode() {

        robot.getOpModeData(telemetry,hardwareMap);
        robot.init("motors");
        robot.init("servos");
        robot.conveyorStatus("start");
        prompt(telemetry, "Init", "HW initialized");
        /*
        robot.mtrConveyorL = hardwareMap.dcMotor.get("left_conveyor");
        robot.mtrConveyorR = hardwareMap.dcMotor.get("right_conveyor");
        robot.mtrConveyorL.setDirection(DcMotor.Direction.FORWARD);
        robot.mtrConveyorR.setDirection(DcMotor.Direction.REVERSE);
        robot.mtrConveyorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.mtrConveyorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        waitForStart();

        while(opModeIsActive()) {
            /*
            double speed = NV60_SPEED;
            if(gamepad1.a) {
                speed *= -1;
                sleep(500);
            }
            robot.mtrConveyorL.setPower(speed);
            robot.mtrConveyorR.setPower(speed);
            */

            setChannels();
            setPowers();

            //TODO make better solution for speed switching
            speedSwitch();

            //linearSlide();
            //extruderControl();
            resets();

            if(gamepad1.y) {
                robot.mtrLinear.setPower(NV60_SPEED);
                linearRun = true;
                linUp = true;
                linDown = false;
            }
            if(gamepad1.a) {
                robot.mtrLinear.setPower(-NV60_SPEED);
                linearRun = true;
                linDown = true;
                linUp = false;
            }

            if(linearRun) {
                if(linUp) {
                    if(robot.mtrLinear.getCurrentPosition() >= SIX_INCHES_NV60) {
                        robot.mtrLinear.setPower(0);
                        robot.mtrLinear.setMode(DcMotor.RunMode.RESET_ENCODERS);
                        linUp = false;
                        linearRun = false;
                    }
                }
                else if (linDown) {
                    if(robot.mtrLinear.getCurrentPosition() <= -SIX_INCHES_NV60) {
                        robot.mtrLinear.setPower(0);
                        robot.mtrLinear.setMode(DcMotor.RunMode.RESET_ENCODERS);
                        linDown = false;
                        linearRun = false;
                    }
                }
            }

            powFL *= modifierValue;
            powFR *= modifierValue;
            powBL *= modifierValue;
            powBR *= modifierValue;


            telemetry.addData("modifier value", modifierValue);
            //telemetry.addData("RunSlow", runSlow);

            telemetry.update();
            /*
            robot.mtrFL.setPower(powFL);
            robot.mtrFR.setPower(powFR);
            robot.mtrBL.setPower(powBL);
            robot.mtrBR.setPower(powBR);*/
            //sleep(125);
        }
    }

    private void prompt(Telemetry telemetry, String prefix, String cmd) {
        telemetry.addData(prefix, cmd);
        telemetry.update();
    }
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

    private void setPowers() {
        powFL = ch2 + ch1 + ch3;
        powFR = ch2 - ch1 - ch3;
        powBL = ch2 - ch1 + ch3;
        powBR = ch2 + ch1 - ch3;

    }

    private void speedSwitch() {
        if(gamepad1.left_bumper) {
            runSlow = true;
        }
        else if(gamepad1.right_bumper) {
            runFast = true;
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
        }

        if(runFast) {
            runFast = false;
            runSlow = false;
            if(modifierValue == modifierValueDefault) {
                modifierValue = fastLimit;
            }
            else if(modifierValue == slowLimit || modifierValue == fastLimit) {
                modifierValue = modifierValueDefault;
            }
        }
    }

    private void linearSlide() {
        String dir = null;
        if (gamepad2.y) {
            robot.moveSlide("up");
        } else if (gamepad2.a) {
            robot.moveSlide("down");
        }
    }

    private void extruderControl() {
        if (gamepad2.x) {
            robot.extrudeGlyphs("both");
        }
        else if (gamepad2.b) {
            robot.extrudeGlyphs("top");
        }
        else if (gamepad2.right_stick_button) {
            robot.extrudeGlyphs("bottom");
        }
        else {
            robot.extrudeGlyphs("stop");
        }
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
}


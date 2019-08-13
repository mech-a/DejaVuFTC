package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Motor Run/Counts", group="Internal")

public class akullinearslideproblem extends LinearOpMode {

    DcMotor liftMotor;

    public static void goUp(int ticks, DcMotor motor, double power) {
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        int currPos = motor.getCurrentPosition();
        motor.setTargetPosition(ticks /* - currPos */);
        motor.setPower(power);
    }

    public static void goDown(int ticks, DcMotor motor, double power) {
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        int currPos = motor.getCurrentPosition();
        motor.setTargetPosition(ticks /* - currPos */);
        motor.setPower(power);
    }

    @Override
    public void runOpMode() {

        liftMotor = hardwareMap.dcMotor.get("lift_motor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (opModeIsActive()) {

            double stage = 1;
            boolean goingUp;
            boolean goingDown;
            int ticksPerRevolution = 1440;
            double power = 0.5;

            /*
             * Raises the lift, if it is under the third stage
             */
            if(!liftMotor.isBusy()){

                goingUp=false;
                goingDown=false;
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(gamepad1.dpad_up) {
                    if (stage < 3) {
                        stage++;
                        goUp(ticksPerRevolution, liftMotor, power);
                        telemetry.addData("Going to stage: ", stage);
                        goingUp = true;
                    }
                    else {
                        break;
                    }
                }

                else if(gamepad1.dpad_down){
                    if (stage > 1) {
                        stage--;
                        goDown(ticksPerRevolution, liftMotor, power);
                        telemetry.addData("Going to stage: ", stage);
                        goingDown=true;
                    }
                    else{
                        break;
                    }
                }
            }

            /* else if (liftMotor.isBusy()){

                if ( gamepad1.dpad_up && goingUp ) {
                    stage--;
                    goDown(ticksPerRevolution, liftMotor, power);
                }

                else if (gamepad1.dpad_down && goingDown){
                    stage++;
                    goUp(ticksPerRevolution, liftMotor, power);
                }
            }
           */
        }
    }
}
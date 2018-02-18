package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.called.AutonHandler;
import org.firstinspires.ftc.teamcode.called.HWRobot;

@Autonomous(name="FrontBlue Iterative", group="ITA")
//@Disabled
public class FrontBlueNew extends OpMode
{
    // Declare OpMode members.
    AutonHandler a = new AutonHandler();
    HWRobot r = new HWRobot();
    String team = "blue";
    String area = "eff-front";


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        a.autonInit(telemetry,hardwareMap);
        r.makeActive(true);
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        a.auton(team,area,telemetry,hardwareMap,true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        r.makeActive(false);
    }

}
package org.firstinspires.ftc.teamcode.Interpreter;

import com.google.gson.Gson;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.nio.charset.Charset;

/**
 * Created by gbhat on 3/7/2019.
 */

public abstract class DejaVuLinearOpMode extends LinearOpMode {
    private FTPHandler ftp;
    //TODO set homepath
    private String homePath;
    protected ExampleAuton auton;
    private String fileName;
    private Gson gson;


    protected void download(String fname) {
        fileName = fname;
        try {
            ftp = new FTPHandler();
            ftp.downloadFile(fname, homePath+fname);
            //TODO make log.d
            //System.out.println("FTP File downloaded successfully");
            telemetry.addData("FTP", "File downloaded successfully");
            ftp.disconnect();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    protected void convertJSONToBOJO() throws IOException {
        String file = FileUtils.readFileToString(new File(homePath+fileName), Charset.defaultCharset());
        gson = new Gson();
        auton = gson.fromJson(file, ExampleAuton.class);
        telemetry.addData("JSON To BOJO", "Successful");
    }


}

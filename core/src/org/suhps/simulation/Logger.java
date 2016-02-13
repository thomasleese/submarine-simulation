package org.suhps.simulation;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.utils.Disposable;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * A class which you can use to log the simulation.
 */
public class Logger implements Disposable {

    private static final String TAG = "Logger";

    private FileWriter mWriter;

    public Logger() {
        try {
            SimpleDateFormat dt = new SimpleDateFormat("yyyyMMdd hhmmss");

            String path = System.getProperty("user.home") + "/Desktop";
            path += "/Sub " + dt.format(new Date()) + ".csv";

            Gdx.app.log(TAG, "Writing CSV file to: " + path);
            mWriter = new FileWriter(path);
            mWriter.append("Time,X,Y,Angle\n");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void dispose() {
        try {
            mWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void log(float time, float x, float y, float angle) {
        try {
            mWriter.append(String.valueOf(time));
            mWriter.append(",");
            mWriter.append(String.valueOf(x));
            mWriter.append(",");
            mWriter.append(String.valueOf(y));
            mWriter.append(",");
            mWriter.append(String.valueOf(angle));
            mWriter.append("\n");
            mWriter.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}

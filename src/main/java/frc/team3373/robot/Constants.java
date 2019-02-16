/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import org.json.JSONArray;
import org.json.JSONObject;

/**
 * Add your docs here.
 */
public class Constants {
    private static final String path = "/home/lvuser/config/constants.json";
    private static JSONObject constantsObject;
    
    public static boolean initialized = false;

    public static void loadConstants() throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(new File(path)));
        String st = "";
        String bst = "";
        while((bst = br.readLine()) != null) {
            st = st + bst;
        }
        br.close();
        constantsObject = new JSONObject(st);
        if (constantsObject != null) {
            initialized = true;
        }
    }

    public static void saveConstants() throws IOException {
        BufferedWriter bw = new BufferedWriter(new FileWriter(new File(path),false));
        String jsonString = constantsObject.toString();
        bw.write(jsonString);
        bw.close();
    }

    public static void writeNumber(String name, double value) {
        constantsObject.put(name, value);
    }

    public static void writeString(String name, String value) {
        constantsObject.put(name, value);
    }

    public static void writeNumberArray(String name, int index1, int index2, double value) {
        constantsObject.getJSONArray(name).getJSONArray(index1).put(index2, value);
    }

    public static void writeNumberArray(String name, double[] values) {
        JSONArray array = constantsObject.getJSONArray(name);
        for (int i = 0; i < values.length; i++) {
            array.put(i, values[i]);
        }
    }

    public static void writeNumberArray(String name, double[][] values) {
        JSONArray array = constantsObject.getJSONArray(name);
        for (int i = 0; i < values.length; i++) {
            for (int j = 0; j < values[i].length; j++) {
                array.getJSONArray(i).put(j, values[i][j]);
            }
        }
    }

    public static double getNumber(String name) {
        return constantsObject.getDouble(name);
    }

    public static String getString (String name) {
        return constantsObject.getString(name);
    }

    public static JSONArray getArray(String name) {
        return constantsObject.getJSONArray(name);
    }
}

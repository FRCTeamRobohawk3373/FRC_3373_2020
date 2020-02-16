/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.channels.FileChannel;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Add your docs here.
 */
public class Config {
    private static final String path = "/home/lvuser/config/constants.json";
    private static final String backupPath = "/home/lvuser/config/backup-constants.json";
    private static final String defaultsPath = Filesystem.getDeployDirectory() + "/defaults.json";
    // private static final Object JSONArray = null;
    private static JSONObject constantsObject;

    public static boolean initialized = false;
    private static boolean isBackup = false;

    public static void loadConstants() throws IOException { // Reads constants.json and creates a JSON object
        BufferedReader br;
        try {
            br = new BufferedReader(new FileReader(new File(path)));
        } catch (IOException e) {
            System.err.println("Failed to load constants, loading defaults");
            loadDefaults();
            return;
        }
        String st = "";
        String bst = "";
        while ((bst = br.readLine()) != null) {
            st = st + bst;
        }
        br.close();
        constantsObject = new JSONObject(st);
        if (constantsObject != null) {
            initialized = true;
        }
        display();
    }

    public static void loadDefaults() throws IOException {
        BufferedReader br = new BufferedReader(new FileReader(new File(defaultsPath)));
        String st = "";
        String bst = "";
        while ((bst = br.readLine()) != null) {
            st = st + bst;
        }
        br.close();
        constantsObject = new JSONObject(st);
        if (constantsObject != null) {
            initialized = true;
        }
        display();
    }

    public static void saveConstants() throws IOException { // Copies constants.json to backup-constants.json and copies
                                                            // Json object to constants.json
        if (!isBackup) {
            try {
                copy(path, backupPath);
            } catch (NullPointerException e) {
                System.out.println("No File, Creating New");
            }
        } else {
            isBackup = false;
        }
        File f = new File(path);
        if (!f.getParentFile().exists()) {
            f.getParentFile().mkdirs();
        }
        BufferedWriter bw = new BufferedWriter(new FileWriter(f, false));
        String jsonString = constantsObject.toString();
        bw.write(jsonString);
        bw.close();
    }

    public static void restoreBackup() throws IOException { // Copies backup-constants.json to constants.json
        copy(backupPath, path);
        loadConstants();
        isBackup = true;
    }

    private static void copy(String sourcePath, String destPath) throws IOException { // Method to copy files
        FileChannel sourceChannel = null;
        FileChannel destChannel = null;
        FileInputStream sourceStream = null;
        FileOutputStream destStream = null;
        try {
            sourceStream = new FileInputStream(new File(sourcePath));
            destStream = new FileOutputStream(new File(destPath));
            sourceChannel = sourceStream.getChannel();
            destChannel = destStream.getChannel();
            destChannel.transferFrom(sourceChannel, 0, sourceChannel.size());
        } finally {
            sourceChannel.close();
            destChannel.close();
            sourceStream.close();
            destStream.close();
        }
    }

    public static void writeNumber(String name, double value) { // Writes a number to the JSON object and displays it
        constantsObject.put(name, value);
        NetworkTableInstance.getDefault().getTable("Constants").getEntry(name).setNumber(value);
    }

    public static void removeValue(String name) { // Removes a number from the JSON object
        constantsObject.remove(name);
        NetworkTableInstance.getDefault().getTable("Constants").delete(name);
    }

    /*
     * public static void writeString(String name, String value) {
     * constantsObject.put(name, value); }
     * 
     * public static void writeNumberArray(String name, int index1, int index2,
     * double value) {
     * constantsObject.getJSONArray(name).getJSONArray(index1).put(index2, value); }
     * 
     * public static void writeNumberArray(String name, double[] values) { JSONArray
     * array = constantsObject.getJSONArray(name); for (int i = 0; i <
     * values.length; i++) { array.put(i, values[i]); } }
     * 
     * public static void writeNumberArray(String name, double[][] values) {
     * JSONArray array = constantsObject.getJSONArray(name); for (int i = 0; i <
     * values.length; i++) { for (int j = 0; j < values[i].length; j++) {
     * array.getJSONArray(i).put(j, values[i][j]); } } }
     */

    public static double getNumber(String name, double defaultValue) { // Returns a number from the JSON object
        try {
            return constantsObject.getDouble(name);
        } catch (JSONException e) {
            return defaultValue;
        }
    }

    public static double getNumber(String name) {
        return getNumber(name, 0);
    }

    public static String getString(String name, String defaultValue) {
        try {
            return constantsObject.getString(name);
        } catch (JSONException e) {
            return defaultValue;
        }
    }

    public static String getString(String name) {
        return getString(name, "");
    }

    public static JSONArray getArray(String name) {
        try {
            return constantsObject.getJSONArray(name);
        } catch (JSONException e) {
            return new JSONArray();
        }
    }

    public static void display() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Constants");
        String[] names = JSONObject.getNames(constantsObject);
        for (String name : names) {
            if (!(constantsObject.get(name) instanceof JSONArray)) {
                table.getEntry(name).setValue(constantsObject.get(name));
            } /*else {
                if (!(constantsObject.getJSONArray(name).get(0) instanceof JSONArray)) {
                    JSONArray array = constantsObject.getJSONArray(name);
                    NetworkTable subTable = table.getSubTable(name);
                    for (int i = 0; i < array.length(); i++) {
                        subTable.getEntry(Integer.toString(i)).setDouble(array.getDouble(i));
                    }
                } else {
                    JSONArray array = constantsObject.getJSONArray(name);
                    NetworkTable subTable = table.getSubTable(name);
                    for (int i = 0; i < array.length(); i++) {
                        JSONArray array2 = array.getJSONArray(i);
                        NetworkTable subTable2 = subTable.getSubTable(Integer.toString(i));
                        for (int j = 0; j < array2.length(); j++) {
                            subTable2.getEntry(Integer.toString(j)).setDouble(array2.getDouble(j));
                        }
                    }
                }
            }*/
        }
    }

    public static void updateValues() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Constants");
        String[] keys = JSONObject.getNames(constantsObject);
        for (String key : keys) {
            if (!(constantsObject.get(key) instanceof JSONArray)) {
                constantsObject.put(key, table.getEntry(key).getDouble(-1));
            } /*else {
                if (!(constantsObject.getJSONArray(key).get(0) instanceof JSONArray)) {
                    JSONArray array = constantsObject.getJSONArray(key);
                    NetworkTable subTable = table.getSubTable(key);
                    for (int i = 0; i < array.length(); i++) {
                        array.put(i, subTable.getEntry(Integer.toString(i)));
                    }
                } else {
                    JSONArray array = constantsObject.getJSONArray(key);
                    NetworkTable subTable = table.getSubTable(key);
                    for (int i = 0; i < array.length(); i++) {
                        JSONArray array2 = array.getJSONArray(i);
                        NetworkTable subTable2 = subTable.getSubTable(Integer.toString(i));
                        for (int j = 0; j < array2.length(); j++) {
                            array2.put(j, subTable2.getEntry(Integer.toString(j)).getDouble(-1));
                        }
                    }
                }
            }*/
        }
    }
}
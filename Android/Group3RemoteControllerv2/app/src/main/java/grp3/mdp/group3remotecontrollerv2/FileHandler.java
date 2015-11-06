package grp3.mdp.group3remotecontrollerv2;

import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.PrintWriter;


/**
 * Created by sch-zah on 9/30/2015.
 */
public class FileHandler {

    private File path;
    private String filename;

    private File f;


    public FileHandler(File path,String filename)
    {
        this.path = path;
        this.filename = filename;

        f = new File(path,filename);

    }

    public String readFile()
    {
        String result = "";

        try
        {
            FileReader fr = new FileReader(f);
            BufferedReader br = new BufferedReader(fr);

            result = br.readLine();
        }
        catch (Exception e)
        {

        }

        return result;
    }

    public void writeFile(String msg)
    {
        try
        {
            FileWriter fw = new FileWriter(f,false);
            PrintWriter pw = new PrintWriter(fw);

            pw.write(msg);

            pw.close();
            fw.close();
        }
        catch (Exception e)
        {

        }

    }


}


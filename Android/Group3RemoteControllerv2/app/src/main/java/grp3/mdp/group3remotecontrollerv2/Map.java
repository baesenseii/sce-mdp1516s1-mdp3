package grp3.mdp.group3remotecontrollerv2;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.WindowManager;

/**
 * Created by sch-zah on 10/1/2015.
 */
public class Map {

    private Grid[][] mg;
    private int robotX;
    private int robotY;
    private int mode;
    private String robotOrientation;
    private String obstacleCodes;


    public Map (Grid[][] mg,String obstacleCodes)
    {
        this.mg = mg;
        this.obstacleCodes = obstacleCodes;
        robotOrientation = "south";
        robotX = 1;
        robotY = 1;
        mode = 0;
        generateCoordinates();

    }

    public void setRobotOrientation(int bearing)
    {
        if (bearing == 0)
            robotOrientation = "east";
        else if (bearing == 90)
            robotOrientation =  "north";
        else if (bearing == 180)
            robotOrientation = "west";
        else
            robotOrientation = "south";
    }
    public void setObstacleCode(String obst)
    {
        obstacleCodes = obst;
    }

    public void setMode(int mode)
    {
        this.mode = mode;
    }

    private void generateCoordinates()
    {
        for (int i=0;i<15;i++)
        {
            for (int j=0;j<20;j++)
            {
                int grX = i;
                int grY = 19 - j;
                mg[i][j] = new Grid(grX,grY,"U");

            }
        }

        setObstacles(obstacleCodes);
        setRobot(robotX,robotY);



    }



    private void setObstacles(String obsHex)
    {
        for (int i=0;i<obsHex.length(); i++)
        {
            char hexChar = obsHex.charAt(i);
            int code =  Integer.parseInt(Character.toString(hexChar),16);
            for(int j = 0; j<2; j++){
                int index = i * 2 + j;
                int x = (int) Math.floor( index / 20 );
                int y = index % 20;
                int thisCell = code % 4;
                if(thisCell == 0) // C
                    accessGrid(x, y).setInfo("C");
                if(thisCell == 1) // U
                    accessGrid(x, y).setInfo("U");
                if(thisCell == 2) // U
                    accessGrid(x, y).setInfo("O");
                code = (int) Math.floor(code / 4);
            }
        }
    }


//        int coordX = 0;
//        int coordY = 19;
//
//        for (int i=0;i<obstHex.length(); i++)
//        {
//            char hexChar = obstHex.charAt(i);
//            int value =  Integer.parseInt(Character.toString(hexChar),16);
//            Log.e("value", Integer.toString(value));
//            int counter = 4;
//
//
//
//                while (counter > 0)
//                {
//                    if (coordX == 15)
//                    {
//                        coordX = 0;
//                        coordY--;
//                    }
//
//
//                    double power = counter-1.0;
//                    int mask = (int)Math.pow(2.0,power);
//                    Log.e("Mask", Integer.toString(mask));
//                    int result = mask & value;
//                    Log.e("Result", Integer.toString(result));
//                    if (result == mask)
//                    {
//                        accessGrid(coordX,coordY).setInfo("obstacle");
//                    }
//
//                    counter--;
//                    coordX++;
//                }
//
//        }

    public void refresh()
    {
        generateCoordinates();
    }




    public void setRobot(int startX, int startY)
    {
        Log.e("Robot co-ordinates", "x:" + startX + "\ty:" + startY);
        try {

            robotX = startX;
            robotY = startY;


            accessGrid(robotX, robotY).setInfo("robot"); //center
            accessGrid(robotX+1, robotY).setInfo("robot");
            accessGrid(robotX-1, robotY).setInfo("robot");
            accessGrid(robotX, robotY+1).setInfo("robot");
            accessGrid(robotX, robotY-1).setInfo("robot");
            accessGrid(robotX+1, robotY+1).setInfo("robot");
            accessGrid(robotX-1, robotY-1).setInfo("robot");
            accessGrid(robotX + 1, robotY-1).setInfo("robot");
            accessGrid(robotX-1, robotY+ 1).setInfo("robot");

            //determine orientation

            if (robotOrientation.equals("east"))
            {
                accessGrid(robotX+1,robotY).setInfo("robot-head");
            }
            else if (robotOrientation.equals("west"))
            {
                accessGrid(robotX-1,robotY).setInfo("robot-head");
            }
            else if (robotOrientation.equals("north"))
            {
                accessGrid(robotX,robotY+1).setInfo("robot-head");
            }
            else if (robotOrientation.equals("south"))
            {
                accessGrid(robotX,robotY-1).setInfo("robot-head");
            }
        }
        catch (NullPointerException ne)
        {

        }

    }

    public Grid accessGrid(int x, int y)
    {
        Grid found = null;

        for (int i=0;i<15;i++)
        {
            for (int j=0;j<20;j++)
            {
                if (mg[i][j].getX() == x && mg[i][j].getY() == y)
                {
                    found = mg[i][j];
                }
            }
        }

        return found;
    }








}

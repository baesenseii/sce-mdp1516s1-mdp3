package grp3.mdp.group3remotecontrollerv2;


/**
 * Created by sch-zah on 10/1/2015.
 */
public class Grid {
    private int x; //x-coordinate
    private int y;
    private String info;

    public Grid(int x, int y,String info) {
        this.info = info;
        this.x = x;
        this.y = y;
    }

    public void setX(int x)
    {
        this.x = x;
    }
    public void setY(int y)
    {
        this.y = y;
    }

    public void setInfo(String a)
    {
        info = a;
    }

    public int getX()
    {
        return x;
    }

    public int getY()
    {
        return y;
    }

    public String getInfo()
    {
        return info;
    }
}
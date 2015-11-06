package grp3.mdp.group3remotecontrollerv2;



import android.os.Handler;
import android.os.Message;
import android.os.StrictMode;
import android.util.Log;
import java.io.DataOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.net.Socket;

/**
 * Created by sch-zah on 10/14/2015.
 */
public class WifiHandler {

    private String ip;
    private int port;

    private InputStream is;
    private OutputStream os;
    private DataOutputStream dos;
    private Socket s;

    //in addition to creating the class, make sure permission for Android INTERNET is enabled and added inside the Android Manifest

    public WifiHandler()
    {

        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);


    }

    public boolean isConnectionAlive()
    {
        boolean result = false;

        if (s != null)
        {
            result = s.isConnected();
        }

        return result;
    }
    public void setConnection(String ip, int port)
    {
        this.ip = ip;
        this.port = port;
    }

    public void open()
    {
        try
        {
            s = new Socket(ip,port);

            os = s.getOutputStream();
            is = s.getInputStream();
            dos = new DataOutputStream(s.getOutputStream());

        }
        catch (Exception e)
        {
            Log.d("ERROR", e.toString());
        }
    }

    public void close()
    {
        try
        {
            dos.close();
            is.close();
            os.close();
            s.close();
        }
        catch (Exception e) {}

    }

    public void sendCommand(String msg)
    {
        char[] val = null;

        if (msg.equals("explore"))
            val = new char[]{0x40};
        else if (msg.equals("balance"))
            val = new char[]{0x20};
        else if (msg.equals("align"))
            val = new char[]{0x30};
        else if (msg.equals("fp"))
            val = new char[]{0x50};


        try
        {
            OutputStreamWriter ow = new OutputStreamWriter(os);
            ow.write(val);
            ow.flush();
        }
        catch (Exception e)
        {
            Log.d("ERROR", e.toString());
        }
    }

    public String recvData()
    {
        String result = "";

        try
        {
            InputStreamReader ir = new InputStreamReader(is);

            while (!ir.ready()){}

            char[] buffer = new char[2048];

            int length = ir.read(buffer);

            for (int i = 0;i<length;i++ )
            {
                result += buffer[i];
            }

        }
        catch (Exception e)
        {
            Log.e("ERROR IN RECV", e.toString());
        }
        return result;
    }

    public void requestData(Handler h)
    {
        byte[] data = hexStringToByteArray("10");

        try
        {
            dos.write(data);
        }
        catch (Exception e)
        {

        }

        Message m = Message.obtain();
        m.what = 0;
        m.setTarget(h);
        m.sendToTarget();
    }


    private static byte[] hexStringToByteArray(String s) {
        int len = s.length();
        byte[] data = new byte[len / 2];
        for (int i = 0; i < len; i += 2) {
            data[i / 2] = (byte) ((Character.digit(s.charAt(i), 16) << 4)
                    + Character.digit(s.charAt(i+1), 16));
        }
        return data;
    }


}


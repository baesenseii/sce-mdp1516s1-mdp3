package grp3.mdp.group3remotecontrollerv2;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Set;
import java.util.UUID;
import java.util.regex.Pattern;

/**
 * Created by sch-zah on 9/29/2015.
 */
public class BluetoothHandler {

    private InputStream inStr;
    private OutputStream outStr;
    private BluetoothAdapter btAdapter;
    private BluetoothDevice btTarget;
    private BluetoothSocket btSock;
    private BluetoothServerSocket btServSock;

    private UUID uuid;
    private String service;

    private boolean connect_flag = false;
    private boolean rpi_flag = false;

    private ArrayAdapter<String> btArrayAdapter;

    public BluetoothHandler()
    {
        service = "Test Service";
        uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
        btAdapter = BluetoothAdapter.getDefaultAdapter();

    }

    public void setBluetoothTarget(String mac)
    {
        btTarget = btAdapter.getRemoteDevice(mac);
    }

    public BluetoothAdapter getBtAdapter()
    {
        return btAdapter;
    }

    public boolean getConnection()
    {
        return connect_flag;
    }

    public String[][] getBondedDevicesAddr()
    {
        Set<BluetoothDevice> b = btAdapter.getBondedDevices();

        String[][] mac_addr = new String[b.size()][2];

        int i = 0;

        for (Iterator<BluetoothDevice> it = b.iterator(); it.hasNext();)
        {
            BluetoothDevice temp = it.next();

            mac_addr[i][0] = temp.getName();
            mac_addr[i][1] = temp.getAddress();
            i++;
        }


        return mac_addr;

    }

    public Intent enable()
    {
        Intent enableBT = null;

        if (!btAdapter.isEnabled())
        {
            enableBT = new Intent(btAdapter.ACTION_REQUEST_ENABLE);
        }

        return enableBT;
    }

    public void btDiscoveryMode(int mode)
    {
        if (mode == 1)
        {
            btAdapter.startDiscovery();
        }
        else if (mode == 2)
        {
            btAdapter.cancelDiscovery();
        }
    }

    public void connect(int mode)
    {
        if (mode == 1) //Slave (Client)
        {
            ConnectThread ct = new ConnectThread(btTarget);
            ct.run();
            connect_flag = true;


        }

        else if (mode == 0) //Master (Server)
        {
            try
            {
                btServSock = btAdapter.listenUsingRfcommWithServiceRecord(service,uuid);
                btSock = btServSock.accept();
                btSock.connect();
                connect_flag = true;
            }
            catch (Exception e)
            {

            }

        }

    }

    public int sendData(String msg)
    {
        try
        {
            outStr = btSock.getOutputStream();
            byte[] buffer = msg.getBytes();
            outStr.write(buffer);
        }

        catch(IOException e)
        {
            return -1;
        }

        return 0;
    }

    public void requestData(Handler h)
    {
        try
        {
            byte[] buff = hexStringToByteArray("10");
            outStr = btSock.getOutputStream();
            outStr.write(buff);
        }
        catch (Exception e)
        {

        }
    }

    public void recvData(Handler h)
    {
        int choice = 0;
        String msg = "";

        try
        {
            inStr = btSock.getInputStream();

            while (inStr.available() > 0)
            {
                int data = inStr.read();

                char val = (char)data;

                if (val == '\n')
                {
                    break;
                }
                else
                {
                    msg = msg + val;

                }
            }

        /*
        This is just for demonstration of updating statuses
         */
            if (msg.substring(0,5).equals("Stat:"))
            {
                choice = 1;
            }


            else if (msg.substring(2,6).equals("grid"))
            {
                choice = 2;
            }
            else if (msg.substring(0,1).equals("{"))
            {
                choice = 3;
            }
        /*
        This is just for demonstration of updating statuses
         */
        }
        catch (Exception e)
        {

        }

        //Put regular expression here



        Message mesg_obj = Message.obtain();

        mesg_obj.obj = msg;
        mesg_obj.what = choice;
        mesg_obj.setTarget(h);
        mesg_obj.sendToTarget();



    }

    /*
    *
    * FOR THREADING BLUETOOTH CONNECTIONS [START]
    *
     */
    private class ConnectThread extends Thread {

        private final BluetoothDevice mmDevice;

        public ConnectThread(BluetoothDevice device)
        {
            mmDevice = device;

            try
            {
                btSock = device.createRfcommSocketToServiceRecord(uuid);

            }
            catch (IOException e) { } //mSocket = tmp;
        }

        public void run()
        {
            btAdapter.cancelDiscovery();

            try
            {
                btSock.connect();
            }
            catch (IOException connectException)
            {

                try
                {
                    btSock.close();
                }
                catch (IOException closeException) { }

            }
        }
        public void cancel() {
            try {
                btSock.close();
            } catch (IOException e) { }
        }
    }




/*
*
* FOR BLUETOOTH PAIRING METHODS [END]
*
*/
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

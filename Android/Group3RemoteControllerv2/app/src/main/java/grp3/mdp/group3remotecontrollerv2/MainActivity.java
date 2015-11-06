package grp3.mdp.group3remotecontrollerv2;

import android.app.AlertDialog;
import android.app.Dialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.drawable.Drawable;
import android.net.wifi.WifiManager;
import android.os.Handler;
import android.os.Message;
import android.preference.PreferenceManager;
import android.support.v7.app.ActionBarActivity;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.RelativeLayout;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.GridLayout;

import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;



public class MainActivity extends ActionBarActivity {

    private BluetoothHandler bh = new BluetoothHandler();
    private WifiHandler wh = new WifiHandler();
    private TextView receivedTextBox;
    private Button bluetoothbutton,coordinatesbutton,sendreceivebutton,controlbutton,updateleftrowbutton,sendBtn,upBtn,leftBtn,rightBtn,downBtn,updateCoordBtn,explorebtn,fastestbtn,balancebtn,alignbtn,connection7btn,connection8btn;
    private EditText sendText,xcoord,ycoord;
    private Switch tiltSw;
    private BluetoothAdapter btAdapter;
    private TiltHandler th;
    private ImageView gridmap;
    private Map map;
    private Boolean robotSet;
    private Grid[][] mapGrids;


    BluetoothDevice selectedDevice;
    ArrayList<BluetoothDevice> arrayListBluetoothDevices = new ArrayList<BluetoothDevice>();
    ArrayList<BluetoothDevice> arrayListPairedBluetoothDevices = new ArrayList<BluetoothDevice>();

    ListView PairedDevices ;
    ListView NewDevices;

    private ArrayAdapter<String> mArrayAdapter,mPairedArrayAdapter;
    private ArrayList list,pairedlist;
    private Set<BluetoothDevice> pairedDevices,BluetoothDevices;

    private Boolean wifiUp = false;

    public Dialog dialog;


    private final Handler wfHandler = new Handler()
    {
        public void handleMessage (Message msg)
        {
            final int what = msg.what;
            String output = wh.recvData();

            switch (what)
            {
                case 0:
                    try
                    {
                        JSONObject json = new JSONObject(output);
                        String mapObs = json.getString("map");
                        int newX = json.getJSONObject("robot").getInt("x") - 1;
                        int newY = json.getJSONObject("robot").getInt("y") - 1;
                        int bearing = json.getJSONObject("robot").getInt("degree");

                        map.setObstacleCode(mapObs);
                        map.setRobot(newX, newY);
                        map.setRobotOrientation(bearing);
                        map.refresh();
                        displayMap();
                    }
                    catch (Exception e)
                    {

                    }

                    break;


            }
        }
    };

    private final Handler btHandler = new Handler()
    {
        public void handleMessage (Message msg)
        {
            final int what = msg.what;

            switch (what)
            {
                case 0:
                    if ((String)msg.obj != "")
                    {
                        String val = (String)msg.obj;
                        receivedTextBox.setText(val);
                    }
                    break;
                case 3:
                    try
                    {
                        String output = (String)msg.obj;
                        JSONObject json = new JSONObject(output);
                        String mapObs = json.getString("map");
                        int newX = json.getJSONObject("robot").getInt("x") - 1;
                        int newY = json.getJSONObject("robot").getInt("y") - 1;
                        int bearing = json.getJSONObject("robot").getInt("degree");

                        map.setObstacleCode(mapObs);
                        map.setRobot(newX, newY);
                        map.setRobotOrientation(bearing);
                        map.refresh();
                        displayMap();
                    }
                    catch (Exception e)
                    {

                    }



            }
        }
    };

    private final Handler uiHandler = new Handler()
    {
        public void handleMessage (Message msg)
        {
            final int what = msg.what;

            switch (what)
            {
                case 0:
                    displayMap();
                    break;
            }
        }
    };



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        final View bottomrightlayoutmanual = findViewById(R.id.bottom_rightlayoutmanual);
        final View bottomrightlayoutcoordinates = findViewById(R.id.bottom_rightlayoutcoordinates);
        final View bottomrightlayoutsendreceive = findViewById(R.id.bottom_rightlayoutsendreceive);
        final View bottomrightlayoutauto = findViewById(R.id.bottom_rightlayoutauto);
        final View bottomblanklayout = findViewById(R.id.bottom_rightlayoutblank);



        receivedTextBox = (TextView) findViewById(R.id.receivedTextBox);
        xcoord = (EditText) findViewById(R.id.Xtextbox);
        ycoord = (EditText) findViewById(R.id.Ytextbox);

        bluetoothbutton = (Button) findViewById(R.id.bluetoothbutton);
        //coordinatesbutton = (Button) findViewById(R.id.coordinatesbutton);
        sendreceivebutton = (Button) findViewById(R.id.sendreceivebutton);
        controlbutton = (Button) findViewById(R.id.controlbutton);
        //updateleftrowbutton = (Button) findViewById(R.id.updateleftrowbutton);
        sendBtn = (Button)findViewById(R.id.send_button);
        upBtn = (Button)findViewById(R.id.upButton);
        leftBtn = (Button)findViewById(R.id.leftbutton);
        rightBtn = (Button)findViewById(R.id.rightbutton);
        downBtn = (Button)findViewById(R.id.downbutton);
        updateCoordBtn = (Button)findViewById(R.id.updatecoordbutton);
        sendText = (EditText)findViewById(R.id.commandTextBox);
        tiltSw = (Switch)findViewById(R.id.switchTilt);
        gridmap = (ImageView) findViewById(R.id.imgV);
        explorebtn = (Button)findViewById(R.id.explorebutton);
        fastestbtn = (Button)findViewById(R.id.fastestsearchbutton);
        alignbtn = (Button)findViewById(R.id.alignbutton);
        balancebtn = (Button)findViewById(R.id.balancebutton);
        connection7btn = (Button)findViewById(R.id.connection7button);
        connection8btn = (Button)findViewById(R.id.connection8button);

        Grid[][] mapGrids = new Grid[15][20];
        String obstacle = "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
        map = new Map(mapGrids,obstacle);
        displayMap();



        th = new TiltHandler(getApplicationContext(),btHandler);




        sendBtn.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                bh.sendData(sendText.getText().toString());
                sendText.setText("");
            }
        });

        sendText.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                sendText.setText("");
            }
        });

        final Animation fadeOut = AnimationUtils.loadAnimation(MainActivity.this, R.anim.fadeout);
        final Animation fadeIn = AnimationUtils.loadAnimation(MainActivity.this, R.anim.fadein);

        list = new ArrayList();
        pairedlist = new ArrayList();

        btAdapter = bh.getBtAdapter();

        if (bh.enable() != null)
            startActivityForResult(bh.enable(),0);

        tiltSw.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v)
            {
                if (tiltSw.isChecked())
                {
                    th.runReadings();
                }
                else if (!tiltSw.isChecked())
                {
                    th.stopReading();
                }
            }

        });



        bluetoothbutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                wifiUp = false;
                dialog = new Dialog(MainActivity.this);
                dialog.setCanceledOnTouchOutside(true);
                //dialog.setContentView(R.layout.bluetooth_list);

                AlertDialog.Builder builder = new AlertDialog.Builder(MainActivity.this);
                final AlertDialog alertDialog = builder.create();
                LayoutInflater inflater = getLayoutInflater();
                View convertView = (View) inflater.inflate(R.layout.bluetooth_list, null);
                alertDialog.setView(convertView);
                //alertDialog.setTitle("List");
                PairedDevices = (ListView) convertView.findViewById(R.id.lvwPaired);
                NewDevices = (ListView) convertView.findViewById(R.id.bluetoothlistview);

                mArrayAdapter = new ArrayAdapter<String>(MainActivity.this,R.layout.bluetooth_entry,list);
                mPairedArrayAdapter = new ArrayAdapter<String>(MainActivity.this,R.layout.bluetooth_entry,pairedlist);

                PairedDevices.setAdapter(mPairedArrayAdapter);
                NewDevices.setAdapter(mArrayAdapter);
                getPairedDevices();
                getNewDevices();
                alertDialog.show();

                PairedDevices.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                    public void onItemClick(AdapterView<?> parent, View view, int position, long rowId) {
                        bh.btDiscoveryMode(2);
                        selectedDevice = arrayListPairedBluetoothDevices.get(position);
                        bh.setBluetoothTarget(selectedDevice.getAddress());
                        bh.connect(1);
                        //bh.connect(selectedDevice);
                        // int editedPosition = position + 1;
                        //String selectedFromList = (String) (NewDevices.getItemAtPosition(position));
                        Toast.makeText(MainActivity.this, "Connecting to : " + selectedDevice.getName(), Toast.LENGTH_SHORT).show();
                        //IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
                        //registerReceiver(discoveryResult, filter);
                        alertDialog.dismiss();

                    }
                });

                NewDevices.setOnItemClickListener(new AdapterView.OnItemClickListener() {
                    public void onItemClick(AdapterView<?> parent, View view, int position, long rowId) {
                        bh.btDiscoveryMode(2);
                        selectedDevice = arrayListBluetoothDevices.get(position);
                        bh.setBluetoothTarget(selectedDevice.getAddress());
                        bh.connect(1);
                        //bh.connect(selectedDevice);
                        // int editedPosition = position + 1;
                        //String selectedFromList = (String) (NewDevices.getItemAtPosition(position));
                        Toast.makeText(MainActivity.this, "Connecting to : " + selectedDevice.getName(), Toast.LENGTH_SHORT).show();
                        //IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
                       // registerReceiver(discoveryResult, filter);
                        alertDialog.dismiss();
                    }
                });



            }
        });

        connection7btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                if (wh != null && wh.isConnectionAlive())
                {
                    wh.close();
                }

                wh.setConnection("192.168.3.3",12347);
                wh.open();
                wifiUp = true;
                wh.requestData(wfHandler);


            }
        });

        connection8btn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (wh != null && wh.isConnectionAlive())
                {
                    wh.close();
                }
                wh.setConnection("192.168.3.3",12348);
                wh.open();
                wifiUp = true;
                wh.requestData(wfHandler);


            }
        });


        /*coordinatesbutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                fadeOut.setAnimationListener(new Animation.AnimationListener() {
                    @Override
                    public void onAnimationStart(Animation animation) {
                    }

                    @Override
                    public void onAnimationRepeat(Animation animation) {
                    }

                    @Override
                    public void onAnimationEnd(Animation animation) {
                        bottomrightlayoutmanual.setVisibility(View.GONE);
                    }
                });

                bottomrightlayoutcoordinates.startAnimation(fadeIn);
                bottomrightlayoutcoordinates.setVisibility(View.VISIBLE);
                bottomrightlayoutsendreceive.setVisibility(View.INVISIBLE);
                bottomrightlayoutmanual.setVisibility(View.INVISIBLE);
                bottomrightlayoutauto.setVisibility(View.INVISIBLE);
                bottomblanklayout.setVisibility(View.INVISIBLE);


            }
        }); */

        sendreceivebutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                fadeOut.setAnimationListener(new Animation.AnimationListener() {
                    @Override
                    public void onAnimationStart(Animation animation) {
                    }

                    @Override
                    public void onAnimationRepeat(Animation animation) {
                    }

                    @Override
                    public void onAnimationEnd(Animation animation) {
                        bottomrightlayoutmanual.setVisibility(View.GONE);
                    }
                });

                bottomrightlayoutsendreceive.startAnimation(fadeIn);
                bottomrightlayoutsendreceive.setVisibility(View.VISIBLE);
                bottomrightlayoutcoordinates.setVisibility(View.INVISIBLE);
                bottomrightlayoutmanual.setVisibility(View.INVISIBLE);
                bottomrightlayoutauto.setVisibility(View.INVISIBLE);
                bottomblanklayout.setVisibility(View.INVISIBLE);


            }
        });

        controlbutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                fadeOut.setAnimationListener(new Animation.AnimationListener() {
                    @Override
                    public void onAnimationStart(Animation animation) {
                    }

                    @Override
                    public void onAnimationRepeat(Animation animation) {
                    }

                    @Override
                    public void onAnimationEnd(Animation animation) {
                        bottomrightlayoutmanual.setVisibility(View.GONE);
                    }
                });

                bottomrightlayoutmanual.startAnimation(fadeIn);
                bottomrightlayoutmanual.setVisibility(View.VISIBLE);
                bottomrightlayoutcoordinates.setVisibility(View.INVISIBLE);
                bottomrightlayoutsendreceive.setVisibility(View.INVISIBLE);
                bottomrightlayoutauto.setVisibility(View.INVISIBLE);
                bottomblanklayout.setVisibility(View.INVISIBLE);



            }
        });

       /* updateleftrowbutton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                fadeOut.setAnimationListener(new Animation.AnimationListener() {
                    @Override
                    public void onAnimationStart(Animation animation) {
                    }

                    @Override
                    public void onAnimationRepeat(Animation animation) {
                    }

                    @Override
                    public void onAnimationEnd(Animation animation) {
                        bottomrightlayoutmanual.setVisibility(View.GONE);
                    }
                });

                bottomrightlayoutauto.startAnimation(fadeIn);
                bottomrightlayoutauto.setVisibility(View.VISIBLE);
                bottomrightlayoutmanual.setVisibility(View.INVISIBLE);
                bottomrightlayoutcoordinates.setVisibility(View.INVISIBLE);
                bottomrightlayoutsendreceive.setVisibility(View.INVISIBLE);
                bottomblanklayout.setVisibility(View.INVISIBLE);


            }
        }); */


        xcoord.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

               xcoord.setText("");
            }
        });

        ycoord.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                ycoord.setText("");
            }
        });

        updateCoordBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                int varX = Integer.valueOf(xcoord.getText().toString());
                int varY = Integer.valueOf(ycoord.getText().toString());
                String data = ("Start of coordinates : (" + varX + "," + varY + ")");
                bh.sendData(data);
                map.setRobot(varX, varY);
            }
        });

       /* upBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                map.moveForward(uiHandler);

            }
        });

        leftBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                map.turnLeft(uiHandler);

            }
        });

        rightBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                map.turnRight(uiHandler);

            }
        });

        downBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                map.turnLeft(uiHandler);
                map.turnLeft(uiHandler);

            }
        }); */

        explorebtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (wifiUp)
                {
                    wh.sendCommand("explore");

                    new Timer().schedule(new TimerTask(){
                        @Override
                        public void run()
                        {
                            wh.requestData(wfHandler);
                        }
                    },0,500);
                }
                else
                {
                    bh.sendData("@");

                    new Timer().schedule(new TimerTask() {
                        @Override
                        public void run() {
                            bh.requestData(btHandler);
                        }
                    }, 0, 500);


                }


            }
        });

        alignbtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (wifiUp)
                    wh.sendCommand("align");
                else
                    bh.sendData("0");

            }
        });

        fastestbtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (wifiUp)
                    wh.sendCommand("fp");
                else
                    bh.sendData("P");

            }
        });

        balancebtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (wifiUp)
                    wh.sendCommand("balance");
                else
                    bh.sendData(" ");
                    bh.requestData(btHandler);

            }
        });



        getApplicationContext().registerReceiver(connResult, new IntentFilter(BluetoothDevice.ACTION_ACL_DISCONNECTED));

        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                runOnUiThread(new Runnable() {
                    public void run() {
                        bh.recvData(btHandler);
                    }
                });
            }
        },0,100);
    }


    private void getPairedDevices() {


        pairedDevices = btAdapter.getBondedDevices();
        mPairedArrayAdapter.clear();

        if (pairedDevices.size() > 0) {
            for (BluetoothDevice bt : pairedDevices)
                pairedlist.add("Name : " + bt.getName() + "\n\nAddress : " + bt.getAddress());
            arrayListPairedBluetoothDevices.addAll(pairedDevices);


        }

        else{

            pairedlist.add("No paired devices right now");
        }


        PairedDevices.setAdapter(mPairedArrayAdapter);

    }

    private void getNewDevices() {
        mArrayAdapter.clear();
        arrayListBluetoothDevices.clear();
//                mBluetoothAdapter.cancelDiscovery();
        bh.btDiscoveryMode(2);
//                mBluetoothAdapter.startDiscovery();
        bh.btDiscoveryMode(1);
        Toast.makeText(MainActivity.this, "Discovering Bluetooth Devices...", Toast.LENGTH_SHORT).show();
        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        registerReceiver(discoveryResult, filter);



    }

    public void displayMap() {


        Bitmap bitmap = Bitmap.createBitmap(520,520, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);
        gridmap.setImageBitmap(bitmap);

        // Rectangle

        Paint paint = new Paint();
        int sqSize = 25;
        float leftx = 10;
        float topy =  10;
        float rightx = leftx + sqSize;
        float bottomy = topy + sqSize;


        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 20; j++) {
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(5);
                paint.setColor(Color.BLACK);

                canvas.drawRect(leftx, topy, rightx, bottomy, paint);

                paint.setColor(Color.WHITE);

                Grid temp = map.accessGrid(i, 19-j);

                if (temp.getInfo().equals("robot"))
                {
                    paint.setColor(Color.YELLOW);
                }
                else if (temp.getInfo().equals("robot-head"))
                {
                    paint.setColor(Color.BLUE);
                }
                else if (temp.getInfo().equals("O"))
                {
                    paint.setColor(Color.RED);
                }
                else if (temp.getInfo().equals("U"))
                {
                    paint.setColor(Color.GRAY);
                }
                else
                {
                    paint.setColor(Color.WHITE);
                }

                paint.setStyle(Paint.Style.FILL);
                canvas.drawRect(leftx, topy, rightx, bottomy, paint);

                topy = bottomy;
                bottomy = bottomy + sqSize;
            }

            topy = 10;
            bottomy = topy + sqSize;
            leftx = rightx;
            rightx = rightx + sqSize;


        }


    }


    private final BroadcastReceiver connResult = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action))
            {
                bh.connect(1);
            }
        }
    };

    private final BroadcastReceiver discoveryResult= new BroadcastReceiver(){


        @Override
        public void onReceive(Context context, Intent intent) {

            String action = intent.getAction(); //may need to chain this to a recognizing function
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {

                BluetoothDevice remoteDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);


                list.add("Name : " + remoteDevice.getName() + "\n\nAddress : " + remoteDevice.getAddress());
               // BluetoothDevices.add(list);
                arrayListBluetoothDevices.add(remoteDevice);
                NewDevices.setAdapter(mArrayAdapter);

            }

            else if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
                Toast.makeText(MainActivity.this, selectedDevice.getName() + " Device is now connected", Toast.LENGTH_LONG).show();
            }

            else if(BluetoothDevice.ACTION_BOND_STATE_CHANGED.equals(action)){
                BluetoothDevice remoteDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                switch (remoteDevice.getBondState()) {
                    case BluetoothDevice.BOND_BONDING:
                        Log.d("BlueToothTestActivity", "it is pairing");
                        //connectionstatus.setText("CONNECTED TO : selectedDevice.getName()");
                       // Toast.makeText(getApplicationContext(), "Connectingffg to : " + selectedDevice.getName(), Toast.LENGTH_SHORT).show();

                        break;
                    case BluetoothDevice.BOND_BONDED:
                        Log.d("BlueToothTestActivity", "finish");
                        Toast.makeText(getApplicationContext(), "Connected to : " + remoteDevice.getName(), Toast.LENGTH_SHORT).show();

                        break;
                    case BluetoothDevice.BOND_NONE:
                        Log.d("BlueToothTestActivity", "cancel");
                    default:
                        break;
                }
            }




//            else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
//
//                Toast.makeText(getApplicationContext(), "Disconnected" + remoteDevice.getName(), Toast.LENGTH_SHORT).show();
////                lastKnownMac = remoteDevice.getAddress();
////                isConnected = true;
//            }

            //Toast.makeText(getApplicationContext(), "Bluetooth Connected" + remoteDevice.getName(), Toast.LENGTH_SHORT).show();
            //lastKnownMac = remoteDevice.getAddress();

            // TODO Auto-generated method stub
            // String action = intent.getAction();
            //if(BluetoothDevice.ACTION_FOUND.equals(action)) {
            //BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            //  list.add(device.getName() + "\n" + device.getAddress());

            //}


            /*if (BluetoothDevice.ACTION_ACL_CONNECTED.equalsIgnoreCase( action )) {
                //Do something if connected
                Toast.makeText(getApplicationContext(), "Bluetooth Connected" + remoteDevice.getName(), Toast.LENGTH_SHORT).show();


              //  intent = new Intent(MainActivity.this, StartCoordinates.class);
               // intent.putExtra("EXTRA_DEVICE_NAME", selectedDevice.getName());
               // intent.putExtra("EXTRA_DEVICE_ADDRESS", selectedDevice.getAddress());
//                startActivityForResult(intent, 1);
            }*/





        }};





}

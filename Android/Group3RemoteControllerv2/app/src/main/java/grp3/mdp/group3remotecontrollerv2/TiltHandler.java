package grp3.mdp.group3remotecontrollerv2;


import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class TiltHandler
{
    private SensorManager mSensorMgr;
    private Sensor acc;
    private float[] tiltData;

    private Handler h;


    public TiltHandler(Context c, Handler h) {
        this.h = h;
        mSensorMgr = (SensorManager)c.getSystemService(Context.SENSOR_SERVICE);
        acc = mSensorMgr.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);



    }

    //    public void getTiltData()
//    {
//        String x = Float.toString(tiltData[0]);
//        String y = Float.toString(tiltData[1]);
//        String z = Float.toString(tiltData[2]);
//        Log.e("Readings","X:"+ x + "\tY:"+y+"\tZ:"+z);
//    }
    public void runReadings()
    {
        mSensorMgr.registerListener(listen,acc,SensorManager.SENSOR_DELAY_GAME);

    }

    public void stopReading()
    {
        mSensorMgr.unregisterListener(listen,acc);

    }

    private final SensorEventListener listen = new SensorEventListener() {
        public void onSensorChanged(SensorEvent e) {

            Message msg = Message.obtain();


            final float[] vals = e.values, target;

            int x = (int)vals[0];
            int y = (int)vals[1];
            int z = (int)vals[2];

            String cmd = "";


         /*
            This is to drive the robot with the phone facing upwards
         */
//                if (y >= 0 && y < 3)
//                {
//                    if (x > 0)
//                    {
//                        Log.e("Tilt Direction","Left");
//                    }
//                    else if (x < 0)
//                    {
//                        Log.e("Tilt Direction","Right");
//                    }
//                }
//                else if (y > 3)
//                {
//                    Log.e("Tilt Direction","Back");
//                }
//                else if (y < 0)
//                {
//                    Log.e("Tilt Direction","Front");
//                }

                /*

                 */


            if (y >= 0 && y < 3)
            {
                if (x > 2)
                    Log.e("Tilt Direction","Back");
                    //cmd = "b"
                else if (x != 0 && x < -2) {
                    Log.e("Tilt Direction", "Front");
                    cmd = "forward";

                }
                else
                    cmd = "stop";
            }

            else if (y > 3)
            {
                Log.e("Tilt Direction","Right");
                cmd = "right";

            }
            else if (y < -3)
            {
                Log.e("Tilt Direction","Left");
                cmd = "left";
            }

            msg.obj = cmd;
            msg.what =2 ;
            msg.setTarget(h);
            msg.sendToTarget();


//            if (vals[2] > vals[1] && vals[2] > vals[0])
//            {
//                Log.e("Tilt Direction","Left");
//            }
//
//            else if (vals[0] > vals[1] && vals[0] > vals[2])
//            {
//                Log.e("Tilt Direction","Right");
//            }

                Log.e("Readings", "X:" + Integer.toString(x) + "\tY:" + Integer.toString(y) + "\tZ:" + Integer.toString(z));
        }





        public void onAccuracyChanged(Sensor event, int res) {}
    };


}

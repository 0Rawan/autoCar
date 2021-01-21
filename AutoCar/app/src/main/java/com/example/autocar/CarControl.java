package com.example.autocar;

import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.util.UUID;

public class CarControl extends AppCompatActivity implements View.OnClickListener {

    //SPP UUID. Look for it
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    // Button btnOn, btnOff, btnDis;
    Button forward, backward, left, right, stop, start, pause;
    EditText etLocation;
    String address = null;
    String location = null;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    private ProgressDialog progress;
    private boolean isBtConnected = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (Build.VERSION.SDK_INT >= 21) {
            Window window = this.getWindow();
            window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
            window.clearFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS);
            window.setStatusBarColor(this.getResources().getColor(R.color.colorPrimaryDark));
        }

        Intent newint = getIntent();
        address = newint.getStringExtra(MainActivity.EXTRA_ADDRESS); //receive the address of the bluetooth device

        //view of the ledControl
        setContentView(R.layout.control_activity);

        //call the widgets
        forward = findViewById(R.id.forward_btn);
        backward = findViewById(R.id.backward_btn);
        left = findViewById(R.id.left_btn);
        right = findViewById(R.id.right_btn);
        stop = findViewById(R.id.stop_btn);
        start = findViewById(R.id.start_btn);
        etLocation = findViewById(R.id.location_edit);
        pause = findViewById(R.id.pause_btn);

        new ConnectBT().execute(); //Call the class to connect

        forward.setOnClickListener(this);
        backward.setOnClickListener(this);
        left.setOnClickListener(this);
        right.setOnClickListener(this);
        stop.setOnClickListener(this);


    }

    @Override
    public void onClick(View v) {

        switch ((v.getId())) {
            case R.id.forward_btn:
                moveForward();      //method to turn on
                break;
            case R.id.backward_btn:
                moveBackward();   //method to turn off
                break;
            case R.id.left_btn:
                moveLeft();
                break;
            case R.id.right_btn:
                moveRight();
                break;

            case R.id.stop_btn:
                Disconnect(); //close connection
                break;

            case R.id.start_btn:
                start(); //start movement
                break;
            case R.id.pause_btn:
                pause();
                break;


        }

    }

    private void Disconnect() {
        if (btSocket != null) //If the btSocket is busy
        {
            try {
                btSocket.close(); //close connection
            } catch (IOException e) {
                msg("Error");
            }
        }
        finish(); //return to the first layout

    }

    private void moveBackward() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("0".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void moveForward() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("1".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void moveLeft() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("2".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void pause() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("5".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void moveRight() {
        if (btSocket != null) {
            try {
                btSocket.getOutputStream().write("3".toString().getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        }
    }

    private void start() {

        location = etLocation.getText().toString();
        if (btSocket != null && location != null) {
            try {
                btSocket.getOutputStream().write(location.getBytes());
            } catch (IOException e) {
                msg("Error");
            }
        } else {
            Toast.makeText(this, "Enter Location", Toast.LENGTH_LONG).show();
        }
    }


    // fast way to call Toast
    private void msg(String s) {
        Toast.makeText(getApplicationContext(), s, Toast.LENGTH_LONG).show();
    }

    private class ConnectBT extends AsyncTask<Void, Void, Void>  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected

        @Override
        protected void onPreExecute() {
            progress = ProgressDialog.show(CarControl.this, "Connecting...", "Please wait!!!");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try {
                if (btSocket == null || !isBtConnected) {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available
                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            } catch (IOException e) {
                ConnectSuccess = false;//if the try failed, you can check the exception here
            }
            return null;
        }

        @Override
        protected void onPostExecute(Void result) //after the doInBackground, it checks if everything went fine
        {
            super.onPostExecute(result);

            if (!ConnectSuccess) {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            } else {
                msg("Connected.");
                isBtConnected = true;
            }
            progress.dismiss();
        }
    }
}
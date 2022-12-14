/*
 * Copyright 2018 Google LLC. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.ar.sceneform.samples.promar;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.media.Image;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import android.util.Base64;
import android.util.Log;
import android.util.SizeF;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.Surface;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.google.ar.core.Trackable;
import com.google.ar.core.DepthPoint;
import com.google.ar.core.Frame;
import com.google.ar.core.Anchor;
import com.google.ar.core.Camera;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.exceptions.NotYetAvailableException;
import com.google.ar.sceneform.AnchorNode;
import com.google.ar.sceneform.math.Vector3;
import com.google.ar.sceneform.math.Quaternion;
import com.google.ar.sceneform.rendering.ModelRenderable;
import com.google.ar.sceneform.samples.promar.env.BorderedText;
import com.google.ar.sceneform.samples.promar.env.ImageUtils;
import com.google.ar.sceneform.samples.promar.env.Size;
import com.google.ar.sceneform.samples.promar.tracking.MultiBoxTracker;
import com.google.ar.sceneform.ux.TransformableNode;

import org.java_websocket.handshake.ServerHandshake;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.io.ByteArrayOutputStream;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Collection;
import java.lang.ref.WeakReference;
import org.java_websocket.client.WebSocketClient;

import com.example.promar.imageprocessinglib.ObjectDetector;
import com.example.promar.imageprocessinglib.model.BoxPosition;
import com.example.promar.imageprocessinglib.model.ImageFeature;
import com.example.promar.imageprocessinglib.model.Recognition;

// main activity of Promar Android Demo
/**
 * This is an example activity that uses the Sceneform UX package to make common AR tasks easier.
 */
public class PromarMainActivity extends AppCompatActivity implements
        SensorEventListener,
        SavingFeatureDialog.OnFragmentInteractionListener {
    private static final String TAG = "MAIN_DEBUG";
    private static final int OWNER_STATE=1, VIEWER_STATE=2;
    private static final double MIN_OPENGL_VERSION = 3.1;

    private int state=OWNER_STATE;

    private TransformableNode andy;

    private float v_viewangle=60, h_viewangle=48;

    private float VO_dist=0;
    private final float VO_dist_for_viewer=0;
    private float v_dist=0;

    //image recognition object as key, value is a list of image features list recognized as this object by TF.
    //Each element is a distortion robust image feature, sorted as left, right, top and bottom
    private Map<String,List<List<ImageFeature>>> rs;
    private Map<String,List<BoxPosition>> bs; //store position
    Size imgSize;

    private MyArFragment arFragment;
    private ModelRenderable andyRenderable;

    private Session arSession;

    private final float last_chk_time=0;
    private boolean opencvLoaded=false;
    //    private Classifier classifier;
    private ObjectDetector objectDetector;

    private OverlayView trackingOverlay;
    /*** from tensorflow sample code***/
    private Handler handler;
    private long timestamp = 0; //it's actually a counter
    private final Bitmap cropCopyBitmap = null;
    private Bitmap croppedBitmap = null;
    private final Bitmap rgbFrameBitmap=null;
    private Bitmap copyBitmp = null;

    private HandlerThread handlerThread;
    private final byte[][] yuvBytes = new byte[3][];
    private final int[] rgbBytes = null;
    private int yRowStride;

    protected int previewWidth = 0;
    protected int previewHeight = 0;
    private ImageView imgView;

    private Integer sensorOrientation;
    private Matrix frameToCropTransform;
    private Matrix cropToFrameTransform;
    private Matrix frameToDisplayTransform;
    private final int rotation=90;

    private MultiBoxTracker tracker;

    private byte[] luminanceCopy;
    private List<Recognition> recognitions;
    private Boolean has_initialized=false;
    //    private Classifier detector;
    private BorderedText borderedText;

    static Boolean onRecord = false;
    static Boolean onRetrieve = false;
    private WebSocketClient myWebSocketClient;
    private Integer frame_no = 0;
    private Integer device_no = -1;
    private boolean send_vo = false;
    private boolean viewer_vo = false;
    private boolean get_viewer_position = false;
    private float pixel_x, pixel_y, view_x, view_y, view_z, andyRotation;
    private boolean need_relocalize = false;
    private boolean planeVisible = false;
    private final float andyScale = 0.3f;
    private static MyUtils util = new MyUtils(true);

    // private String KalibInString = "737.037,699.167,340.565,218.486";
    private static final String KalibInString = util.deviceToKalib();

    private final PointerDrawable pointer = new PointerDrawable();

    public void toastShow(Context con, String str){
        Toast toast = Toast.makeText(con, str, Toast.LENGTH_SHORT);
        TextView v = toast.getView().findViewById(android.R.id.message);
        v.setTextColor(Color.BLACK);
        toast.show();
    }

    @Override
    @SuppressWarnings({"AndroidApiChecker", "FutureReturnValueIgnored"})
    // CompletableFuture requires api level 24
    // FutureReturnValueIgnored is not valid
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        if (!checkIsSupportedDeviceOrFinish(this)) return;
        OpenCVLoader.initDebug();

        //calculate filed of views
        setFOV();

        setContentView(R.layout.activity_ux);
        arFragment = (MyArFragment) getSupportFragmentManager().findFragmentById(R.id.ux_fragment);
        arFragment.getArSceneView().getPlaneRenderer().setVisible(false);
        arFragment.getInstructionsController().setEnabled(false);
        imgView = findViewById(R.id.imgview);

        //orientation sensor manager
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(
                Sensor.TYPE_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        conn();

        arFragment.setActivity(this);
        arFragment.setOnFrameListener((frameTime, frame) -> {
            Bitmap bitmap=null;
            Image img=null;
            if(frame==null) { Log.d(TAG,"frame is null"); return; }
            try {
                img = frame.acquireCameraImage();
                String msg = img.getFormat()+":"+ img.getWidth() +","+ img.getHeight();
                luminanceCopy = util.imageToByte(img);
                bitmap=util.imageToBitmap(img);
                img.close();
            }
            catch(Exception e){ return; }
            if(objectDetector==null) {
                initTF(bitmap);
                initDistParameters();
            }
            if(!planeVisible) planeVisible = checkPlaneVisibility(frame);
            processImage(bitmap);
        });
        arFragment.setOnTapArPlaneListener((HitResult hitResult, Plane plane, MotionEvent motionEvent) -> {});
        showPointer();
        loadModel();
        setPlaceBtn();
        setRetrieveBtn();
    }

    private void showPointer() {
        View contentView = findViewById(android.R.id.content);
        pointer.setLoc(960,540);
        contentView.getOverlay().add(pointer);
        contentView.invalidate();
    }

    private boolean checkPlaneVisibility(Frame frame) {
        Collection plane = frame.getUpdatedTrackables(Plane.class);
        if(plane.isEmpty()) return false;
        toastShow(PromarMainActivity.this, "plane found");
        return true;
    }

    private void loadModel() {
        WeakReference<PromarMainActivity> weakActivity = new WeakReference<>(this);
        ModelRenderable.builder()
        .setSource(this, Uri.parse("https://github.com/ChexterWang/IRL_AR_Android_2/blob/master/app/sampledata/models/andy.glb?raw=true"))
        .setIsFilamentGltf(true)
        .setAsyncLoadEnabled(true)
        .build()
        .thenAccept(model -> {
            PromarMainActivity activity = weakActivity.get();
            if(activity != null) activity.andyRenderable = model;
        })
        .exceptionally(throwable -> {
            Toast toast = Toast.makeText(this, "Unable to load andy renderable", Toast.LENGTH_LONG);
            toast.setGravity(Gravity.CENTER, 0, 0);
            toast.show();
            return null;
        });
    }

    private void setPlaceBtn() {
        Button recBtn = findViewById(R.id.record);
        recBtn.setOnClickListener(view -> {
            Button btn=(Button)view;
            String tag=(String)btn.getTag();
            if(tag.equals("Place VO")) {
                placeAndy((float)previewWidth/2, (float)previewHeight/2);
                runOnUiThread(()-> {
                    btn.setText(getString(R.string.confirm));
                    btn.setTag("Confirm");
                });
            } else {
                rs=null;//delete previous data
                onRecord = true;
                runOnUiThread(()-> {
                    btn.setText(getString(R.string.place));
                    btn.setTag("Place VO");
                    send_vo = true;
                    toastShow(PromarMainActivity.this, "set host");
                });
            }
        });
        recBtn.setTag("Place VO");
    }

    private void setRetrieveBtn() {
        Button rteBtn = findViewById(R.id.retrieve);
        viewer_vo = true;
        rteBtn.setOnClickListener(view ->
            AsyncTask.execute(() -> {
                Button btn=(Button)view;
                String tag=(String)btn.getTag();
                if(tag.equals("Retrieve")) {
                    get_viewer_position = false;
                    onRetrieve = true;
                    runOnUiThread(()-> {
                        viewer_vo = true;
                        toastShow(PromarMainActivity.this, "viewer send");
                        btn.setText(getString(R.string.clear));
                        btn.setTag("Clear");
                        //btn.setEnabled(false);
                    });
                } else {
                    onRetrieve=false;
                    runOnUiThread(()-> {
                        btn.setText(getString(R.string.retrieve));
                        btn.setTag("Retrieve");
                        if(andy != null) andy.setParent(null);
                    });
                }
            })
        );
        rteBtn.setTag("Retrieve");
        rteBtn.setEnabled(true);
    }

    void setFOV() {
        //suppose there is only one camera
        int camNum = 0;
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            String[] cameraIds = manager.getCameraIdList();
            for (String id : cameraIds) {
                CameraCharacteristics characteristics = manager.getCameraCharacteristics(id);
                int cOrientation = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (cOrientation == CameraCharacteristics.LENS_FACING_BACK) {
                    float[] maxFocus = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS);
                    SizeF size = characteristics.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE);
                    float w = size.getWidth();
                    float h = size.getHeight();
                    h_viewangle = (float) (2 * Math.atan(w / (maxFocus[0] * 2)))/(float)(2*Math.PI)*360;
                    v_viewangle = (float) (2 * Math.atan(h / (maxFocus[0] * 2)))/(float)(2*Math.PI)*360;
                }
            }
        } catch (CameraAccessException e){
            Log.e(TAG, e.getMessage(), e);
        }
    }

    void initDistParameters() {
        float width=previewHeight;
        float height=previewWidth;
        float v_dist_center_x=(float) (width/2/Math.tan(h_viewangle/2/180*Math.PI)); //virtual distance to the center of the cameraview
        float v_dist_center_y=(float) (height/2/Math.tan(v_viewangle/2/180*Math.PI)); //virtual distance to the center of the cameraview
        Log.d("match_strings",String.format("width:%.02f,height:%.02f",width, height));
        Log.d("match_strings","dist_center:"+ v_dist_center_x +" "+ v_dist_center_y);
        //TODO:how about adding v_dist_center_y? Why their value varied so much
        v_dist=v_dist_center_x;//(v_dist_center_x+v_dist_center_y)/2; //distance in units of pixels
    }

    void initTF(Bitmap bitmap) {
        previewWidth = bitmap.getWidth();
        previewHeight = bitmap.getHeight();
        sensorOrientation = rotation - getScreenOrientation();
        objectDetector = new ObjectDetector();
        objectDetector.init(this);

        //if(tracker==null)
        tracker = new MultiBoxTracker(this);

        croppedBitmap = Bitmap.createBitmap(300, 300, Bitmap.Config.ARGB_8888);
        copyBitmp = Bitmap.createBitmap(previewWidth, previewHeight, Bitmap.Config.ARGB_8888);
        frameToCropTransform =
                ImageUtils.getTransformationMatrix(
                        previewWidth, previewHeight,
                        300, 300,
                        sensorOrientation, false);
        cropToFrameTransform = new Matrix();
        frameToCropTransform.invert(cropToFrameTransform);
        trackingOverlay = findViewById(R.id.tracking_overlay);
        trackingOverlay.addCallback((canvas) -> tracker.draw(canvas));
    }

    public void conn()
    {
        URI uri;
        try {
            uri = new URI("ws://192.168.50.76:9090/");
        } catch (URISyntaxException e) {
            e.printStackTrace();
            return;
        }

        myWebSocketClient = new WebSocketClient(uri) {
            @Override
            public void onOpen(ServerHandshake serverHandshake) {
                Log.i("Websocket", "Opened");
                myWebSocketClient.send("Hello from " + Build.MANUFACTURER + " " + Build.MODEL);
            }

            @Override
            public void onMessage(String s) {
                final String message = s;
                new Thread(() -> {
                    String[] entries = message.split(":");
                    String data = entries[3].substring(0, entries[3].length() - 6);
                    String topic = entries[3].substring(2, 4);
                    data = data.substring(data.indexOf('"')+1, data.length() - 2);

                    List<String> poselist = Arrays.asList(data.split(","));
                    if(poselist.get(0).equals("host_set"))
                        runOnUiThread(()-> toastShow(getApplicationContext(), "server set done"));
                    else if(poselist.get(0).equals("viewer_done")){
                        // viewer_done,{pixel[0]:.5f},{pixel[1]:.5f},{pos[0]},{pos[1]},{self.VO.sm_angle:.0f},{device_id}
                        int rcv_id = Integer.parseInt(poselist.get(6));
                        if(rcv_id == device_no){
                            pixel_x = Float.parseFloat(poselist.get(1));
                            pixel_y = Float.parseFloat(poselist.get(2));
                            view_x = Float.parseFloat(poselist.get(3));
                            view_y = Float.parseFloat(poselist.get(4));
                            andyRotation = Float.parseFloat(poselist.get(5));
                            runOnUiThread(() -> toastShow(getApplicationContext(), "rcv angle: "+andyRotation));
                            get_viewer_position = true;
                        }
/*
                        System.out.println("msg data" + data);
                        System.out.println("msg x" + view_x);
                        System.out.println("msg y" + view_y);
                        System.out.println("msg z" + view_z);
                        if(view_z>0.85 && view_z <=1.2f){
                            view_z = 1.2f;
                        }else if(view_z>1.2f && view_z<=1.3f){
                            view_z = 1.4f;
                        }else if(view_z>1.3f && view_z<=1.4f){
                            view_z = 1.5f;
                        }else if(view_z > 1.4f){
                            view_z = 1.6f;
                        }
                        view_z = view_z / 1.2f;
                        if(view_z > 1.4f){
                            view_z = 1.4f;
                        }
                        System.out.println("msg x final" + view_x);
                        System.out.println("msg y final" + view_y);
                        System.out.println("msg z final" + view_z);
*/
                    } else if(poselist.get(0).equals("initialize")) {
                        has_initialized = true;
                        runOnUiThread(() -> toastShow(getApplicationContext(), "SLAM initialize"));
                    } else if(poselist.get(0).equals("relocalize")) {
                        // need_relocalize = true;
                        // no host(viewer -> pre host)
                        onRetrieve = false;
                        viewer_vo = false;
                        // device_no = Integer.valueOf(poselist.get(1));
                        runOnUiThread(() -> {
                            // Toast.makeText(getApplicationContext(), "relocalize", Toast.LENGTH_SHORT).show();
                            toastShow(getApplicationContext(), "server not set yet");
                        });
                    } else if(poselist.get(0).equals("id")) {
                        if(device_no < 1){
                            device_no = Integer.parseInt(poselist.get(1));
                            runOnUiThread(() -> toastShow(getApplicationContext(), "device id: "+poselist.get(1)));
                        }
                    }
                }).start();
            }

            @Override
            public void onClose(int i, String s, boolean b) {
                Log.i("Websocket", "Closed " + s);
            }

            @Override
            public void onError(Exception e) {
                Log.i("Websocket", "Error " + e.getMessage());
            }
        };
        try{
            myWebSocketClient.connect();
        }catch (IllegalStateException e){
            Log.i("websocket error", String.valueOf(e));
        }

    }

    boolean has_subscribed = false;
    void web_sock_send(final String enc_img)
    {
        new Thread(() -> {
            String encoded_img = enc_img.replaceAll("(\\r|\\n|\\r\\n)+", "");
            JSONObject obj = new JSONObject();
            JSONObject obj1 = new JSONObject();
            String data_header;
            // add "_" at end to properly parse data (in IRL_websocket.py)
            if(device_no < 0) {
                data_header = device_no.toString() + '_' + frame_no.toString() + "_getId_" + encoded_img + "_" + KalibInString + "_";
                device_no = 0;
            } else if(send_vo) {
                data_header = device_no.toString() + '_' + frame_no.toString() + "_host_" + encoded_img + "_";
                send_vo = false;
            } else if(viewer_vo && device_no > 0) {
                data_header = device_no.toString() + '_' + frame_no.toString() + "_viewer_" + encoded_img + "_";
                viewer_vo = false;
            } else data_header = device_no.toString() + '_' + frame_no.toString() + "_F_" + encoded_img + "_";

            try {
                obj1.put("data", data_header);
                obj.put("op","publish");
                obj.put("topic","/chatter");
                obj.put("msg", obj1);
            } catch(JSONException e) {
                Log.i("error sending", "gg");
            }
            String data = "";
            String data1 = "";
            String op1 = "advertise";
            String topic1 = "/chatter";
            String type1 = "std_msgs/String";
            data = "{\"op\": \"" + op1 + "\"";
            data += ",\"topic\":\"" + topic1 + "\"";
            data += ",\"type\":\"" + type1 + "\"}";

            if(myWebSocketClient.isOpen()){
                //For handshaking
                // myWebSocketClient.send(data);

                //For subscription bounding box
                if(!has_subscribed) {
                    String op = "subscribe";
                    String id = "001";
                    String topic = "/IRL_SLAM";
                    String type = "std_msgs/String";
                    String subs = "{\"op\": \"" + op + "\"";
                    subs += ",\"id\":\"" + id + "\"";
                    subs += ",\"topic\":\"" + topic + "\"";
                    subs += ",\"type\":\"" + type + "\"}";
                    has_subscribed = true;
                    myWebSocketClient.send(subs);
                }
                myWebSocketClient.send(obj.toString());
            } else {
                // util.logi("no ws access");
            }
            frame_no++;
        }).start();
    }

    void send_image(Bitmap bitmap)
    {
        if(bitmap==null) return;
        ByteArrayOutputStream stream = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.JPEG,80,stream);
        final byte[] byteArray = stream.toByteArray();
        //cast to string
        final String encoded_img = Base64.encodeToString(byteArray, Base64.DEFAULT);
        //create json object
        System.out.println("Keyframe bitmap: row: "+bitmap.getHeight()+"column: "+bitmap.getWidth());
        System.out.println("Keyframe size!"+byteArray.length);
        web_sock_send(encoded_img);
    }

    long timeStamp = 0;
    static private final long kInterval = 500;

    public Bitmap changeSize(Bitmap bitmap){
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        //??????????????????1/2??????
        float screenWidth  = 640;		// ?????????(??????,???:480px)
        float screenHeight = 480;		// ?????????(??????,???:800p)
        float scaleWidth = screenWidth/width;
        float scaleHeight = screenHeight/height;
        // ?????????????????????matrix??????
        Matrix matrix = new Matrix();
        matrix.postScale(scaleWidth, scaleHeight);
        // ??????????????????
        Bitmap newbm = Bitmap.createBitmap(bitmap, 0, 0, width, height, matrix,true);
        return newbm;
    }

    int ticks = 0;
    void processImage(Bitmap bitmap) {
        if(bitmap==null) return;
        if (luminanceCopy == null) return;

        ++timestamp;
        final long currTimestamp = timestamp;

        if(has_initialized && !need_relocalize){
            ticks +=1;
            if(ticks==5){
                send_image(changeSize(bitmap));
                ticks = 0;
            }
        } else {
            send_image(changeSize(bitmap));
            need_relocalize = false;
        }
        
        //stop the background thread when program is halted
        if (System.currentTimeMillis() - timeStamp > kInterval && handler != null) {
            timeStamp = System.currentTimeMillis();
        } else return;
        final Canvas canvas = new Canvas(croppedBitmap);
        canvas.drawBitmap(bitmap, frameToCropTransform, null);
        final Canvas c = new Canvas(copyBitmp);
        c.drawBitmap(bitmap, new Matrix(), null);

        runInBackground(() -> {
            if(onRetrieve) retrieve(copyBitmp);
        });
        bitmap.recycle();
    }

    private float scaleX(float x, float fromX, float toX){ return x*toX/fromX; }

    private void retrieve(Bitmap img) {
        if(get_viewer_position) {
            float screenWidth  = 640;		// ?????????(??????,???:480px)
            float screenHeight = 480;		// ?????????(??????,???:800p)
            float scaleWidth = screenWidth/previewWidth;
            float scaleHeight = screenHeight/previewHeight;
            float ptx = pixel_x / scaleWidth;
            float pty = pixel_y / scaleHeight;
            runOnUiThread(() -> {
                if(placeAndy(ptx, pty)){
                    get_viewer_position = false;
                    onRetrieve=false;
                    return;
                }
                viewer_vo = true;
                get_viewer_position = false;
                onRetrieve = true;
                // placeAndy(view_y, view_x , -view_z);
            });
        }
    }

    //theta: the change of the view angle
    float angleChangeHelper(float d_ro_ori, float d_ro, float scale, float theta){
        float h_ro = v_dist * scale;
        double beta = Math.atan(d_ro/h_ro);
        double alhpa = 90-beta+theta;
        double l = h_ro / Math.cos(beta);
        double m = Math.sqrt(d_ro_ori*d_ro_ori+l*l-2*d_ro_ori*l*Math.cos(alhpa));
        double zeta = Math.acos((m*m+l*l-d_ro_ori*d_ro_ori)/(2*m*l));
        float dx = (float)Math.tan(zeta-beta)*h_ro+d_ro;
        return dx;
    }

    public void onPeekTouch() {}

    private float getDepth(Image depthImage, int x, int y) {
        // The depth image has a single plane, which stores depth for each
        // pixel as 16-bit unsigned integers.
        Image.Plane plane = depthImage.getPlanes()[0];
        int byteIndex = x * plane.getPixelStride() + y * plane.getRowStride();
        ByteBuffer buffer = plane.getBuffer().order(ByteOrder.nativeOrder());
        return (float) buffer.getShort(byteIndex) / 1000;
    }

    private boolean placeAndyWithAnchor(Anchor anchor) {
        AnchorNode anchorNode = new AnchorNode(anchor);
        anchorNode.setParent(arFragment.getArSceneView().getScene());
        if(andy==null) andy = new TransformableNode(arFragment.getTransformationSystem());
        andy.setLocalRotation(Quaternion.axisAngle(new Vector3(0f, 1f, 0f), andyRotation));
        andy.getScaleController().setMinScale(andyScale);
        andy.setLocalScale(new Vector3(andyScale, andyScale, andyScale));
        andy.setParent(anchorNode);
        andy.setRenderable(andyRenderable);
        andy.select();
        return true;
    }

    private boolean placeAndy(float x, float y){

        Frame frame = arFragment.getArSceneView().getArFrame();
        if(frame == null) return false;

        View contentView = findViewById(android.R.id.content);
        pointer.setLoc((int)x,(int)y);
        contentView.getOverlay().add(pointer);
        contentView.invalidate();

        Image depthImage = null;
        float imageDepth = 0;
        float depthLowerBound = 0;
        float depthUpperBound = 0;
        try {
            depthImage = frame.acquireDepthImage16Bits();
            float xScaled = scaleX(x, previewWidth, depthImage.getWidth());
            float yScaled = scaleX(y, previewHeight, depthImage.getHeight());
            imageDepth = getDepth(depthImage, (int)xScaled, (int)yScaled);
            float acceptedRange = (float) 0.15;
            depthLowerBound = imageDepth * (1-acceptedRange);
            depthUpperBound = imageDepth * (1+acceptedRange);
            util.logi("depth image distance: " + imageDepth);
        } catch (NotYetAvailableException e) {
            util.logi("depthImage not available due to no tracking");
        } finally {
            if (depthImage != null) depthImage.close();
        }

        android.graphics.Point pt = new android.graphics.Point((int)x, (int)y);
        List<HitResult> hits;
        hits = frame.hitTest(pt.x, pt.y);
        for (HitResult hit : hits) {
            Trackable tab = hit.getTrackable();
            float hitDepth = hit.getDistance();
            boolean depthInRange = ((hitDepth > depthLowerBound) && (hitDepth < depthUpperBound));
            if(depthImage == null) depthInRange = true;
            util.logi("className: " + tab.getClass().getName() +
                    ", hit distance: " + hitDepth +
                    ", depthImage depth: " + imageDepth +
                    ", in the range:" + depthInRange);
            if(get_viewer_position) {
                //  if(depthInRange) return placeAndy(view_y, view_x, -hitDepth);
               if(depthInRange) return placeAndyWithAnchor(hit.createAnchor());
            } else {
                if(tab instanceof Plane && ((Plane) tab).isPoseInPolygon(hit.getHitPose())) {
                    if(depthInRange) return placeAndyWithAnchor(hit.createAnchor());
                } else if (tab instanceof DepthPoint) return placeAndyWithAnchor(hit.createAnchor());
            }
        }
        return false;
    }

    private boolean placeAndy(float x, float y, float z){
        Frame f = arFragment.getArSceneView().getArFrame();
        if(f == null) return false;
        Camera camera = f.getCamera();
        Pose mCameraRelativePose = Pose.makeTranslation(x, y, z);
        arSession = arFragment.getArSceneView().getSession();
        Pose cPose = camera.getPose().compose(mCameraRelativePose).extractTranslation();
        Anchor anchor=arSession.createAnchor(cPose);
        return placeAndyWithAnchor(anchor);
    }

    private void placeAndy(){ placeAndy(0.0f, 0.0f, 0.0f); }

    private void placeAndyWithDist(float dist){ placeAndy(0.0f, 0.0f, -dist); }


    /**
     * Returns false and displays an error message if Sceneform can not run, true if Sceneform can run
     * on this device.
     *
     * <p>Sceneform requires Android N on the device as well as OpenGL 3.1 capabilities.
     *
     * <p>Finishes the activity if Sceneform can not run
     */
    public static boolean checkIsSupportedDeviceOrFinish(final Activity activity) {
        String openGlVersionString =
                ((ActivityManager) activity.getSystemService(Context.ACTIVITY_SERVICE))
                        .getDeviceConfigurationInfo()
                        .getGlEsVersion();
        if (Double.parseDouble(openGlVersionString) < MIN_OPENGL_VERSION) {
            Log.e(TAG, "Sceneform requires OpenGL ES 3.1 later");
            Toast.makeText(activity, "Sceneform requires OpenGL ES 3.1 or later", Toast.LENGTH_LONG)
                    .show();
            activity.finish();
            return false;
        }
        return true;
    }

    //miniature image view set image
    void setImage(Bitmap image){
        runOnUiThread(() -> imgView.setImageBitmap(image));
    }

    void setImage(Image image){
        if(!opencvLoaded) return;
        Mat mat = util.imageToMat(image);
        //ImageView imgview=findViewById(R.id.imgview);
        Bitmap bitmap = Bitmap.createBitmap(mat.cols(),  mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat,bitmap);
        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        Bitmap rotatedBitmap = Bitmap.createBitmap(bitmap , 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
        //imgview.setImageBitmap(rotatedBitmap);
    }

    private final BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            opencvLoaded = (status == LoaderCallbackInterface.SUCCESS);
            super.onManagerConnected(status);
        }
    };

    @Override
    public void onResume()
    {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCVlibrary not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV libraryfound inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        handlerThread = new HandlerThread("inference");
        handlerThread.start();
        handler = new Handler(handlerThread.getLooper());
        arFragment.onResume();
    }

    public void addCallback(final OverlayView.DrawCallback callback) {
        final OverlayView overlay = findViewById(R.id.debug_overlay);
        if(overlay != null) overlay.addCallback(callback);
    }

    @Override
    public synchronized void onPause() {

        handlerThread.quit();
        try {
            handlerThread.join();
            handlerThread = null;
            handler = null;
        } catch (final InterruptedException e) {
           Log.e(TAG, "Can't stop handler thread");
        }
        arFragment.onPause();

        super.onPause();
    }

    protected synchronized void runInBackground(final Runnable r) {
        if(handler != null) handler.post(r);
    }

    protected int getScreenOrientation() {
        switch (getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_270:
                return 270;
            case Surface.ROTATION_180:
                return 180;
            case Surface.ROTATION_90:
                return 90;
            case Surface.ROTATION_0:
                return 0;
            default:
                return 0;
        }
    }

    public void requestRender() {
        final OverlayView overlay = findViewById(R.id.debug_overlay);
        if(overlay != null) overlay.postInvalidate();
    }

    //Codes below this line is for orientation monitor
    private RotationData refRD =null;
    private RotationData cRD=null;
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;
    private boolean checkAngle=false, firstValue=false;

    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    public void onSensorChanged(SensorEvent event) {
        if (firstValue) {
            refRD = new RotationData(event.values);
            firstValue=false;
        } else {
            cRD=new RotationData(event.values);
            displayData(cRD);
        }
    }

    void displayData(RotationData temp){
        TextView textView=findViewById(R.id.cangle);
        if(refRD != null) textView.setText(getString(R.string.dispData, refRD.toString(), temp.toString()));
    }

    void setAngle(){
        refRD = cRD;    //just in case cRD is re-assigned value
        checkAngle=true;
        firstValue=true;
    }

    @Override
    public void onFragmentInteraction(Uri uri) {}

    private class RotationData{
        private final float x;
        private final float y;
        private final float z;
        private float cos;
        private static final int FROM_RADS_TO_DEGS = -57;

        RotationData(float x, float y, float z){
            this.x = x;
            this.y = y; //
            this.z = z;
        }

        RotationData(float[] values){
            float[] rotationMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(rotationMatrix, values);
            int worldAxisX = SensorManager.AXIS_X;
            int worldAxisZ = SensorManager.AXIS_Z;
            float[] adjustedRotationMatrix = new float[9];
            SensorManager.remapCoordinateSystem(rotationMatrix, worldAxisX, worldAxisZ, adjustedRotationMatrix);
            float[] orientation = new float[3];
            SensorManager.getOrientation(adjustedRotationMatrix, orientation);
            float pitch = orientation[1] * FROM_RADS_TO_DEGS;
            float roll = orientation[2] * FROM_RADS_TO_DEGS;
            float azimuth = orientation[0] * FROM_RADS_TO_DEGS;
            x = pitch;  //top,bottom perspective. -90~90.
            y = roll;   //rotation on same vertical plane. -180~180
            z = azimuth;    //left,right perspective. -180~180
        }

        @NonNull
        public String toString(){
            return String.format(Locale.US, "%.02f %.02f %.02f", x, y, z);
        }
    }
}

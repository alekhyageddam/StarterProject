package com.example.starterproject;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.support.v4.app.ActivityCompat;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.widget.TextView;
import android.widget.Toast;

import com.esri.arcgisruntime.UnitSystem;
import com.esri.arcgisruntime.concurrent.ListenableFuture;
import com.esri.arcgisruntime.geometry.CoordinateFormatter;
import com.esri.arcgisruntime.geometry.GeometryEngine;
import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.geometry.PointCollection;
import com.esri.arcgisruntime.geometry.Polygon;
import com.esri.arcgisruntime.geometry.Polyline;
import com.esri.arcgisruntime.geometry.SpatialReference;
import com.esri.arcgisruntime.geometry.SpatialReferences;
import com.esri.arcgisruntime.layers.ArcGISMapImageLayer;
import com.esri.arcgisruntime.loadable.LoadStatus;
import com.esri.arcgisruntime.mapping.ArcGISMap;
import com.esri.arcgisruntime.mapping.Basemap;
import com.esri.arcgisruntime.mapping.view.DefaultMapViewOnTouchListener;
import com.esri.arcgisruntime.mapping.view.Graphic;
import com.esri.arcgisruntime.mapping.view.GraphicsOverlay;
import com.esri.arcgisruntime.mapping.view.MapView;
import com.esri.arcgisruntime.security.AuthenticationManager;
import com.esri.arcgisruntime.security.DefaultAuthenticationChallengeHandler;
import com.esri.arcgisruntime.security.OAuthConfiguration;
import com.esri.arcgisruntime.symbology.SimpleFillSymbol;
import com.esri.arcgisruntime.symbology.SimpleLineSymbol;
import com.esri.arcgisruntime.symbology.SimpleMarkerSymbol;
import com.esri.arcgisruntime.tasks.networkanalysis.DirectionEvent;
import com.esri.arcgisruntime.tasks.networkanalysis.DirectionManeuver;
import com.esri.arcgisruntime.tasks.networkanalysis.DirectionManeuverType;
import com.esri.arcgisruntime.tasks.networkanalysis.Route;
import com.esri.arcgisruntime.tasks.networkanalysis.RouteParameters;
import com.esri.arcgisruntime.tasks.networkanalysis.RouteResult;
import com.esri.arcgisruntime.tasks.networkanalysis.RouteTask;
import com.esri.arcgisruntime.tasks.networkanalysis.Stop;
import com.esri.arcgisruntime.tasks.networkanalysis.TravelMode;

import java.net.MalformedURLException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private MapView mMapView;
    private GraphicsOverlay mGraphicsOverlay;
    private GraphicsOverlay _personUpdate = new GraphicsOverlay();
    private Point mStart;
    private Point mEnd;
    private LocationManager _locationManager;
    private Location _nextLoc;
    private ArrayList<DirectionManeuver> _directions = new ArrayList<>();
    private boolean nextDirec = false;
    private DirectionManeuverType dmt = DirectionManeuverType.DEPART;
    private boolean newRoute = false;
    private static SensorManager _sensorManager;
    private Sensor _accelerometer;
    private Sensor _magnetometer;
    private float[] _r = new float[9];
    private float[] _orientation = new float[3];
    private float[] _lastAccelerometer = new float[3];
    private float[] _lastMagnetometer = new float[3];
    private boolean _lastAccelerometerSet = false;
    private boolean _lastMagnetometerSet = false;
    private double heading = 0.0;


    @SuppressLint("MissingPermission")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //set up compass
        _sensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        _accelerometer = _sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        _magnetometer = _sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);


        // *** ADD ***
        mMapView = findViewById(R.id.mapView);
        //allows for app to access location
        _locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);
        _nextLoc = _locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        //start location
        startLocation();
        // *** ADD ***
        setupOAuthManager();
        setupMap();
        // *** ADD ***
        createGraphicsOverlay();
        // *** ADD ***
        setupOAuthManager();

    }

    private void setupMap() {
        ArcGISMap map = null;
        if (mMapView != null) {
            Basemap.Type basemapType = Basemap.Type.IMAGERY_WITH_LABELS_VECTOR;
            double latitude = 30.6185;
            double longitude = -96.3365;
            int levelOfDetail = 11;
            map = new ArcGISMap(basemapType, latitude, longitude, levelOfDetail);
            ArcGISMapImageLayer layer1 = new ArcGISMapImageLayer("http://gis.tamu.edu/arcgis/rest/services/FCOR/ADA_120717/MapServer");
            map.getBasemap().getBaseLayers().add(layer1);
            mMapView.setMap(map);
        }

        _lastAccelerometerSet = false;
        _lastMagnetometerSet = false;
        _sensorManager.registerListener(this, _accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        _sensorManager.registerListener(this, _magnetometer, SensorManager.SENSOR_DELAY_NORMAL);
        mMapView.setMap(map);
        // *** ADD ***
//        ArcGISMapImageLayer traffic = new ArcGISMapImageLayer(getResources().getString(R.string.traffic_service));
//        map.getOperationalLayers().add(traffic);

        mMapView.setOnTouchListener(new DefaultMapViewOnTouchListener(this, mMapView) {
            @Override
            public boolean onSingleTapConfirmed(MotionEvent e) {
                android.graphics.Point screenPoint = new android.graphics.Point(
                        Math.round(e.getX()),
                        Math.round(e.getY()));
                Point mapPoint = mMapView.screenToLocation(screenPoint);
                mapClicked(mapPoint);
                return super.onSingleTapConfirmed(e);
            }
        });

    }

    @Override
    protected void onPause() {
        if (mMapView != null) {
            mMapView.pause();
        }
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mMapView != null) {
            mMapView.resume();
        }
        _lastAccelerometerSet = false;
        _lastMagnetometerSet = false;
        //_sensorManager.registerListener(this, _accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
       // _sensorManager.registerListener(this, _magnetometer, SensorManager.SENSOR_DELAY_NORMAL);



    }

    @Override
    protected void onDestroy() {
        if (mMapView != null) {
            mMapView.dispose();
        }
        super.onDestroy();
    }


    private void createGraphicsOverlay() {
        mGraphicsOverlay = new GraphicsOverlay();
        mMapView.getGraphicsOverlays().add(mGraphicsOverlay);
    }


    private void setupOAuthManager() {
        String clientId = getResources().getString(R.string.client_id);
        String redirectUrl = getResources().getString(R.string.redirect_url);

        try {
            OAuthConfiguration oAuthConfiguration = new OAuthConfiguration("https://www.arcgis.com", clientId, redirectUrl);
            DefaultAuthenticationChallengeHandler authenticationChallengeHandler = new DefaultAuthenticationChallengeHandler(this);
            AuthenticationManager.setAuthenticationChallengeHandler(authenticationChallengeHandler);
            AuthenticationManager.addOAuthConfiguration(oAuthConfiguration);
        } catch (MalformedURLException e) {
            showError(e.getMessage());
        }
    }

    private void showError(String message) {
        Log.d("FindRoute", message);
        Toast.makeText(getApplicationContext(), message, Toast.LENGTH_LONG).show();
    }

    private void setMapMarker(Point location, SimpleMarkerSymbol.Style style, int markerColor, int outlineColor) {
        float markerSize = 8.0f;
        float markerOutlineThickness = 2.0f;
        SimpleMarkerSymbol pointSymbol = new SimpleMarkerSymbol(style, markerColor, markerSize);
        pointSymbol.setOutline(new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, outlineColor, markerOutlineThickness));
        Graphic pointGraphic = new Graphic(location, pointSymbol);
        mGraphicsOverlay.getGraphics().add(pointGraphic);
    }

    private void setStartMarker(Point location) {
        mGraphicsOverlay.getGraphics().clear();
        setMapMarker(location, SimpleMarkerSymbol.Style.DIAMOND, Color.rgb(226, 119, 40), Color.BLUE);
        mStart = location;
        mEnd = null;
    }

    private void setEndMarker(Point location) {
        setMapMarker(location, SimpleMarkerSymbol.Style.SQUARE, Color.rgb(40, 119, 226), Color.RED);
        mEnd = location;
        findRoute();
    }

    private void mapClicked(Point location) {
        if (mStart == null) {
            // Start is not set, set it to a tapped location
            setStartMarker(location);
        } else if (mEnd == null) {
            // End is not set, set it to the tapped location then find the route
            setEndMarker(location);
            newRoute = false;
        } else if(newRoute) {
            // Both locations are set; re-set the start to the tapped location
            setStartMarker(location);
        }
    }

    //Add a method to find a route between two locations

    private void findRoute() {
        String routeServiceURI = getResources().getString(R.string.routing_url);
        final RouteTask solveRouteTask = new RouteTask(getApplicationContext(), routeServiceURI);
        solveRouteTask.loadAsync();
        solveRouteTask.addDoneLoadingListener(() -> {
            if (solveRouteTask.getLoadStatus() == LoadStatus.LOADED) {
                final ListenableFuture<RouteParameters> routeParamsFuture = solveRouteTask.createDefaultParametersAsync();
                routeParamsFuture.addDoneListener(() -> {
                    try {
                        RouteParameters routeParameters = routeParamsFuture.get();
                        routeParameters.setReturnDirections(true);
                        routeParameters.setReturnStops(true);
                        routeParameters.setReturnRoutes(true);
                        routeParameters.setDirectionsDistanceUnits(UnitSystem.METRIC);
                        routeParameters.setFindBestSequence(true);

                        routeParameters.setTravelMode(solveRouteTask.getRouteTaskInfo().getTravelModes().get(1));
                        List<Stop> stops = new ArrayList<>();
                        stops.add(new Stop(mStart));
                        stops.add(new Stop(mEnd));
                        routeParameters.setStops(stops);
                        // routeParameters.setFindBestSequence(true);
                        routeParameters.setPreserveFirstStop(true);
                        routeParameters.setPreserveLastStop(true);
                        final ListenableFuture<RouteResult> routeResultFuture = solveRouteTask.solveRouteAsync(routeParameters);
                        routeResultFuture.addDoneListener(() -> {
                            try {
                                RouteResult routeResult = routeResultFuture.get();
                                Route firstRoute = routeResult.getRoutes().get(0);
                                for (int i = 0; i < firstRoute.getDirectionManeuvers().size(); i++) {
                                    _directions.add(firstRoute.getDirectionManeuvers().get(i));
                                }
                                Polyline routePolyline = firstRoute.getRouteGeometry();
                                SimpleLineSymbol routeSymbol = new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, Color.BLUE, 4.0f);
                                Graphic routeGraphic = new Graphic(routePolyline, routeSymbol);
                                mGraphicsOverlay.getGraphics().add(routeGraphic);
                            } catch (InterruptedException | ExecutionException e) {
                                showError("Solve RouteTask failed " + e.getMessage());
                            }
                        });
                    } catch (InterruptedException | ExecutionException e) {
                        showError("Cannot create RouteTask parameters " + e.getMessage());
                    }
                });
            } else {
                showError("Unable to load RouteTask " + solveRouteTask.getLoadStatus().toString());
            }

            //solve the route and select the first returned result
        });


    }
    private double angleFromCoordinate(double lat1, double long1, double lat2,
                                       double long2) {

        double dLon = (long2 - long1);

        double y = Math.sin(dLon) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1)
                * Math.cos(lat2) * Math.cos(dLon);

        double brng = Math.atan2(y, x);

        brng = Math.toDegrees(brng);
        brng = (brng + 360) % 360;
        brng = 360 - brng;
        return brng;
    }
        //start tracking location
    private void startLocation() {
        LocationListener locationListener = new LocationListener() {
            @SuppressLint("DefaultLocale")
            @Override
            public void onLocationChanged(Location location) {

                String corr = String.format("%f,%f", location.getLatitude(), location.getLongitude());
                TextView tv = findViewById(R.id.textView);
                if(location.distanceTo(_nextLoc) < 2){
                    showError("location recognized");
                    nextDirec = true;
                }
               // tv.setText(String.format("Distance to next stop %f m",location.distanceTo(_nextLoc)));
                double opp = _nextLoc.getLongitude() - location.getLongitude();
                double angle = _nextLoc.getBearing() - location.getBearing();
                double bearing = location.bearingTo(_nextLoc);
                bearing= (bearing+360 )%360;


                tv.setText(String.format("Curr Loc: (%f,%f)\n Next Loc: (%f,%f)\n Distance to next loc: %f m\nBearing: %f\n Heading: %f",
                        location.getLatitude(),location.getLongitude(),
                        _nextLoc.getLatitude(),_nextLoc.getLongitude(),
                        location.distanceTo(_nextLoc),
                        bearing,
                        heading));
                Point mapPoint = CoordinateFormatter.fromLatitudeLongitude(corr, mMapView.getSpatialReference());
                SimpleMarkerSymbol symbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, Color.BLUE, 12);
                Graphic graphic = new Graphic(mapPoint, symbol);
                _personUpdate.clearSelection();
                _personUpdate.getGraphics().clear();
                _personUpdate.getGraphics().add(graphic);


            }

            @Override
            public void onStatusChanged(String provider, int status, Bundle extras) {

            }

            @Override
            public void onProviderEnabled(String provider) {

            }

            @Override
            public void onProviderDisabled(String provider) {

            }
        };
        if (ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && ActivityCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }

        mMapView.getGraphicsOverlays().add(_personUpdate);
        _locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 2, 0, locationListener);

    }

    public void initRoute(android.view.View v) {
        nextDirec=true;
        showError("starting route");
        new Thread(() -> startRoute()).start();


    }
    @SuppressLint("MissingPermission")
    private Location parseLoc(String loc){
        String [] tmp = loc.split(" ");
        double lat, lng;
        if(tmp[0].substring(tmp[0].length()-1).equals("N")){
             lat = Double.parseDouble(tmp[0].substring(0,tmp[0].length()-1));
        }else{
            lat = -1*Double.parseDouble(tmp[0].substring(0,tmp[0].length()-1));
        }
        if(tmp[1].substring(tmp[1].length()-1).equals("E")){
            lng = Double.parseDouble(tmp[1].substring(0,tmp[1].length()-1));
        }else{
            lng = -1*Double.parseDouble(tmp[1].substring(0,tmp[1].length()-1));

        }

        Location tmpL = _locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        tmpL.setLatitude(lat);
        tmpL.setLongitude(lng);

        return tmpL;

    }

    private void startRoute() {
        if(_directions.isEmpty()){
            runOnUiThread(()-> {
                showError("no route");
            });
            return;
        }
        String start =  CoordinateFormatter.toLatitudeLongitude(mStart, CoordinateFormatter.LatitudeLongitudeFormat.DECIMAL_DEGREES, 6);
        _nextLoc = parseLoc(start);
        runOnUiThread(() -> {
            //showError(de.getEventText());
            SimpleMarkerSymbol symbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CROSS, Color.BLACK, 12);
            Graphic graphic = new Graphic(mStart, symbol);
            GraphicsOverlay go = new GraphicsOverlay();
            mMapView.getGraphicsOverlays().add(go);
            go.getGraphics().add(graphic);
        });

        while (!_directions.isEmpty()) {

            if (nextDirec) {

                nextDirec = false;
                DirectionManeuver dm = _directions.remove(0);
                for (DirectionEvent de : dm.getDirectionEvents()) {
                    String ltn = CoordinateFormatter.toLatitudeLongitude(de.getGeometry(), CoordinateFormatter.LatitudeLongitudeFormat.DECIMAL_DEGREES, 6);

                    if(_nextLoc == parseLoc(ltn)){
                        showError("same place");
                        break;
                    }
                    runOnUiThread(() -> {

                        SimpleMarkerSymbol symbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CROSS, Color.BLACK, 12);
                        Graphic graphic = new Graphic(de.getGeometry(), symbol);
                        GraphicsOverlay go = new GraphicsOverlay();
                        mMapView.getGraphicsOverlays().add(go);
                        go.getGraphics().add(graphic);
                       // System.out.println(""+(_nextLoc.bearingTo(parseLoc(ltn))+360)%360);

                    });

                   // String ltn = CoordinateFormatter.toLatitudeLongitude(de.getGeometry(), CoordinateFormatter.LatitudeLongitudeFormat.DECIMAL_DEGREES, 6);
                    //String.format("Turn: %s when you arrive at %s",dm.getManeuverType(),ltn);
                    dmt = dm.getManeuverType();
                    _nextLoc = parseLoc(ltn);
                   // System.out.println(_nextLoc.toString());
                }

            }


        }
        String end =  CoordinateFormatter.toLatitudeLongitude(mEnd, CoordinateFormatter.LatitudeLongitudeFormat.DECIMAL_DEGREES, 6);
        _nextLoc = parseLoc(end);
        runOnUiThread(() -> {
            //showError(de.getEventText());
            SimpleMarkerSymbol symbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CROSS, Color.BLACK, 12);
            Graphic graphic = new Graphic(mEnd, symbol);
            GraphicsOverlay go = new GraphicsOverlay();
            mMapView.getGraphicsOverlays().add(go);
            go.getGraphics().add(graphic);
        });
        newRoute = true;
    }
    private double convertRadiantoDegree(double rad){
        double tmp = Math.toDegrees(rad);
        tmp = (tmp+360) %360;
        return tmp;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor == _accelerometer) {
            System.arraycopy(event.values, 0, _lastAccelerometer, 0, event.values.length);
            _lastAccelerometerSet = true;
        } else if (event.sensor == _magnetometer) {
            System.arraycopy(event.values, 0, _lastMagnetometer, 0, event.values.length);
            _lastMagnetometerSet = true;
        }
        if (_lastAccelerometerSet && _lastMagnetometerSet) {
            SensorManager.getRotationMatrix(_r, null, _lastAccelerometer, _lastMagnetometer);
            SensorManager.getOrientation(_r, _orientation);
            heading = convertRadiantoDegree(_orientation[0]);
            Log.i("OrientationTestActivity", String.format("Orientation: z: %f, x: %f, y: %f",
                    convertRadiantoDegree(_orientation[0]), convertRadiantoDegree(_orientation[1]),convertRadiantoDegree(_orientation[2])));
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}

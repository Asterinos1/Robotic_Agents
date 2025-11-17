// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import com.cyberbotics.webots.controller.Accelerometer;
import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.LightSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

import java.util.Random;

public class Rat0 extends Robot {

  protected final int timeStep = 32;
  protected final double maxSpeed = 300;
  
//we're not going to be needing these since we're abandoning Braitenberg Vehicle logic.
//protected final double[] collisionAvoidanceWeights = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//protected final double[] slowMotionWeights = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  protected Accelerometer accelerometer;
  protected Camera camera;
  protected int cameraWidth, cameraHeight;
  protected Motor leftMotor, rightMotor;
  protected DistanceSensor[] distanceSensors = new DistanceSensor[8];
  protected LightSensor[] lightSensors = new LightSensor[8];
  protected LED[] leds = new LED[10];

  public Rat0() {
    
  }

  public void initialize() {
    accelerometer = getAccelerometer("accelerometer");
    camera = getCamera("camera");
    camera.enable(8*timeStep);
    cameraWidth=camera.getWidth();
    cameraHeight=camera.getHeight();
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    for (int i=0;i<10;i++) {
      leds[i]=getLED("led"+i);
    };
    for (int i=0;i<8;i++) {
      distanceSensors[i] = getDistanceSensor("ps"+i);
      distanceSensors[i].enable(timeStep);
      lightSensors[i] = getLightSensor("ls"+i);
      lightSensors[i].enable(timeStep);
    }
    batterySensorEnable(timeStep);
  }

  public void run() {

    int blink = 0;
    int oldDx = 0;
    Random r = new Random();
    boolean turn = false;
    boolean right = false;
    boolean seeFeeder = false;
    double battery;
    double oldBattery = -1.0;
    int image[];
    double distance[] = new double[8];
    int ledValue[] = new int[10];
    double leftSpeed, rightSpeed;
    //if we want to guide e-puck using the left wall we need 
    //to keep track of a few things.
    boolean isCloseToLeftWall = false; //1) if we are close to the left wall
    boolean frontSideBlocked = false; //2) avoid frontal collision
    boolean leftCorner = false; //3) don't miss turns as we move along the wall

    while (step(timeStep) != -1) {
      // read sensor information
      for(int i=0;i<8;i++) distance[i] = distanceSensors[i].getValue();
      battery = batterySensorGetValue();
      for(int i=0;i<10;i++) ledValue[i] = 0;
      //get sensor info on distance for the desired directions
      //distance[4] and distance[3] are in the front
      //also distance[3] will be used for when we there might be an available
      //turn as we traverse across the wall.
      //distance[2] is left side sensor
      frontSideBlocked = distance[4] > 390 || distance[3] > 390;
      isCloseToLeftWall = distance[2] > 300;
      leftCorner = distance[3] > 190;
      leftSpeed = maxSpeed;
      rightSpeed = maxSpeed;
      //setting the movement logic. we must stick to the left side 
      //at all times.
      if(frontSideBlocked){
        leftSpeed = -maxSpeed;
        rightSpeed = maxSpeed;
      }else{ 
        //when close to the left wall, we keep moving forward.
        if(isCloseToLeftWall){
          leftSpeed = maxSpeed;
          rightSpeed = maxSpeed;
        }else{
          //if we're not close enough, we do a soft left turn to get close.
          rightSpeed = maxSpeed/7;
        }
        //if there's an available left turn as we move, we turn to go there.
        if(leftCorner){
          leftSpeed = maxSpeed/2.7;
          rightSpeed = maxSpeed;
        }
      }
      //recharging battery
      if (battery > oldBattery) {
        leftSpeed  = 0.0;
        rightSpeed = 0.0;
        ledValue[8] = 1; // turn on the body led
      }
      oldBattery = battery;
      if (blink++ >= 20) { // blink the back LEDs
        ledValue[4] = 1;
        if (blink == 40) blink = 0;
      }
      
      //set actuators
      for(int i=0; i<10; i++) {
        leds[i].set(ledValue[i]);
      }
      //reversing
      leftMotor.setVelocity(-0.00628 * leftSpeed);
      rightMotor.setVelocity(-0.00628 * rightSpeed);
    } 
    // Enter here exit cleanup code
  }

  public static void main(String[] args) {
    Rat0 rat0 = new Rat0();
    rat0.initialize();
    rat0.run();
  }
}

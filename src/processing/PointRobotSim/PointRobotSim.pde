// imports

import shiffman.box2d.*;
import hypermedia.net.*;

//Global variables;
int now;

int A_R_LB;
int A_R_RB;

int H_R_LB;
int H_R_RB;

Robot a_r;
Robot h_r;

float ROBOT_RADIUS = 10;
float GOAL_RADIUS = 10;

PVector a_r_pos;
PVector h_r_pos;

String a_r_color = "w";
String h_r_color = "gray";


PVector a_r_vel = new PVector(0.0, 0.0);
PVector h_r_vel = new PVector(0.0, 0.0);


//UDP Variables;
UDP a_udp;
String A_DEST_IP = "127.0.0.1";
int A_DEST_PORT = 8025;
int A_HOST_PORT = 6000;

UDP h_udp;
String H_DEST_IP = "127.0.0.1";
int H_DEST_PORT = 8026;
int H_HOST_PORT = 6001;

UDP key_udp;
String KEY_DEST_IP = "127.0.0.1";
int KEY_DEST_PORT = 8027;
int KEY_HOST_PORT = 6002;

//Goals

ArrayList <Goal> a_gList;
ArrayList <Goal> h_gList;

HashMap<Integer, String> goal_shapes = new HashMap<Integer, String>();

StringDict goal_color = new StringDict();


//Bools 

boolean isAGInitialized = false;
boolean isHGInitialized = false;
boolean isARInitialized = false;
boolean isHRInitialized = false;


boolean allInitialized = false;

//General
int TEXT_SIZE = 18;
float delta_t = 100;


void setup()
{
  size(800, 600);
  frameRate(60);
  //smooth();

  A_R_LB = 0;
  A_R_RB = width/2;

  H_R_LB = width/2;
  H_R_RB = width;

  setUpDicts();
  instantiateRobots();
  instantiateGoalLists();
  delay(1000);
  instantiateUDP();
  //thread("sendRobotPoses");
  allInitialized = true;
  now = millis();
}


void draw()
{
  background(0);
  strokeWeight(2);
  stroke(255);
  if (millis () > now + delta_t);
  {
    sendRobotPoses();
    now = millis();
  }
  line(width/2.0, 0, width/2.0, height);
  printText();
  if (isAGInitialized && isHGInitialized)
  { 
    displayGoals();
  }
  if (isARInitialized && isHRInitialized)
  {
    displayRobots();
  }
}

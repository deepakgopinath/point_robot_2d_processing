// imports

import shiffman.box2d.*;
import hypermedia.net.*;

//Global variables;
int now;

int AUTONOMY_ROBOT_LB;
int AUTONOMY_ROBOT_RB;

int HUMAN_ROBOT_LB;
int HUMAN_ROBOT_RB;

Robot autonomy_robot;
Robot human_robot;

float ROBOT_RADIUS = 10;
float GOAL_RADIUS = 10;

PVector autonomy_rob_pos;
PVector human_rob_pos;

String autonomy_rob_color = "w";
String human_rob_color = "gray";


PVector autonomy_rob_vel = new PVector(0.0, 0.0);
PVector human_rob_vel = new PVector(0.0, 0.0);


//UDP Variables;
UDP autonomy_udp;
String AUTONOMY_DEST_IP = "127.0.0.1";
int AUTONOMY_DEST_PORT = 8025;
int AUTONOMY_HOST_PORT = 6000;

UDP human_udp;
String HUMAN_DEST_IP = "127.0.0.1";
int HUMAN_DEST_PORT = 8025;
int HUMAN_HOST_PORT = 6001;

UDP key_udp;
String KEY_DEST_IP = "127.0.0.1";
int KEY_DEST_PORT = 8026;
int KEY_HOST_PORT = 6002;

//Goals

ArrayList <Goal> autonomy_goalList;
ArrayList <Goal> human_goalList;

StringDict goal_color = new StringDict();

//Bools 

boolean isAutonomyGoalInitialized = false;
boolean isHumanGoalInitialized = false;
boolean isAutonomyRobotInitialized = false;
boolean isHumanRobotInitialized = false;


boolean allInitialized = false;

//General
int TEXT_SIZE = 18;


void setup()
{
  size(800, 600);
  frameRate(60);
  
  AUTONOMY_ROBOT_LB = 0;
  AUTONOMY_ROBOT_RB = width/2;

  HUMAN_ROBOT_LB = width/2;
  HUMAN_ROBOT_RB = width;

  smooth();
  
  now = millis();
  
  instantiateRobots();
  instantiateGoalLists();
  delay(1000);
  instantiateUDP();
  allInitialized = true;
}


void draw()
{
  background(0);
  strokeWeight(2);
  stroke(255);
  line(width/2.0, 0, width/2.0, height);
  printText();
  if (isAutonomyGoalInitialized && isHumanGoalInitialized)
  { 
    displayGoals();
  }
  if (isAutonomyRobotInitialized && isHumanRobotInitialized)
  {
    displayRobots();
  }

  if (millis () > now + 100);
  {
    sendRobotPoses();
    now = millis();
  }
}

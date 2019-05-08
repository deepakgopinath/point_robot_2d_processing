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

//Bools 

boolean isGoalInitialized = false;
boolean isRobotInitialized = false;
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

  //autonomy_robot = new Robot((AUTONOMY_ROBOT_LB + AUTONOMY_ROBOT_RB)/2.0, height/2.0, ROBOT_RADIUS, AUTONOMY_ROBOT_LB, AUTONOMY_ROBOT_RB, autonomy_rob_color);
  //human_robot = new Robot((HUMAN_ROBOT_LB + HUMAN_ROBOT_RB)/2.0, height/2.0, ROBOT_RADIUS, HUMAN_ROBOT_LB, HUMAN_ROBOT_RB, human_rob_color);
  autonomy_robot = new Robot(0.0, 0.0, ROBOT_RADIUS, AUTONOMY_ROBOT_LB, AUTONOMY_ROBOT_RB, autonomy_rob_color);
  human_robot = new Robot(0.0, 0.0, ROBOT_RADIUS, HUMAN_ROBOT_LB, HUMAN_ROBOT_RB, human_rob_color);

  autonomy_rob_pos = autonomy_robot.getPosition();
  human_rob_pos = human_robot.getPosition();

  autonomy_rob_vel = autonomy_robot.getVelocity();
  human_rob_vel = human_robot.getVelocity();

  autonomy_goalList = new ArrayList<Goal>();
  human_goalList = new ArrayList<Goal>();

  delay(1000);
  autonomy_udp = new UDP(this, AUTONOMY_HOST_PORT);
  autonomy_udp.listen(true);
  human_udp = new UDP(this, HUMAN_HOST_PORT);
  human_udp.listen(true);
  key_udp = new UDP(this, KEY_HOST_PORT);
  key_udp.listen(true);

  allInitialized = true;
}


void draw()
{
  background(0);
  strokeWeight(2);
  stroke(255);
  line(width/2.0, 0, width/2.0, height);
  printText();
  if (isGoalInitialized)
  { 
    displayGoals();
  }
  if (isRobotInitialized)
  {
    displayRobots();
  }

  if (millis () > now + 100);
  {

    autonomy_rob_pos = autonomy_robot.getPosition();
    String autonomy_rob_pos_message = "ROBOT_POSE," + str(autonomy_rob_pos.x) + "," + str(autonomy_rob_pos.y);
    sendUDP(autonomy_rob_pos_message, AUTONOMY_DEST_IP, AUTONOMY_DEST_PORT, autonomy_udp);
    //human_rob_pos = human_robot.getPosition();
    //String human_rob_pos_message = "humanRobotPosition," + str(human_rob_pos.x) + "," + str(human_rob_pos.y);
    //sendUDP(human_rob_pos_message, HUMAN_DEST_IP, HUMAN_DEST_PORT, human_udp);
    now = millis();
  }
}

void keyPressed()
{
  String message = "unknown";
  if (key == 'G' || key == 'g')
  {
    message = "GOALS_READY";
  }
  if (key == 'R' || key == 'r')
  {
    message = "GOALS_RESET";
  }
  if (key == 'B' || key == 'b')
  {
    message = "BEGIN_TRIAL";
  }
  if (key == 'E' || key == 'e')
  {
    message = "END_TRIAL";
  }
  if (key == 'I' || key == 'i')
  {
    message = "ROBOT_READY";
  }

  println(message);
  sendUDP(message, KEY_DEST_IP, KEY_DEST_PORT, key_udp);
}

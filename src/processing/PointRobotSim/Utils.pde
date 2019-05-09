
void sendRobotPoses()
{
  autonomy_rob_pos = autonomy_robot.getPosition();
  String autonomy_rob_pos_message = "ROBOT_POSE," + str(autonomy_rob_pos.x) + "," + str(autonomy_rob_pos.y);
  sendUDP(autonomy_rob_pos_message, AUTONOMY_DEST_IP, AUTONOMY_DEST_PORT, autonomy_udp);
  human_rob_pos = human_robot.getPosition();
  String human_rob_pos_message = "HUMAN_ROBOT_POSE," + str(human_rob_pos.x) + "," + str(human_rob_pos.y);
  sendUDP(human_rob_pos_message, HUMAN_DEST_IP, HUMAN_DEST_PORT, human_udp);
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

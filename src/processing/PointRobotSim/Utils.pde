
void sendRobotPoses()
{
  a_r_pos = a_r.getPosition();
  String a_r_pos_message = "A_R_POSE," + str(a_r_pos.x) + "," + str(a_r_pos.y);
  sendUDP(a_r_pos_message, A_DEST_IP, A_DEST_PORT, a_udp);

  //h_r_pos = h_r.getPosition();
  //String h_r_pos_message = "H_R_POSE," + str(h_r_pos.x) + "," + str(h_r_pos.y);
  //sendUDP(h_r_pos_message, H_DEST_IP, H_DEST_PORT, h_udp);
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

  sendUDP(message, KEY_DEST_IP, KEY_DEST_PORT, key_udp);
}

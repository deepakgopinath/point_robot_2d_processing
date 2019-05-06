void sendUDP(String message, String ip, int port, UDP udp)
{
  byte[] data = message.getBytes();
  udp.send(data, ip, port);
}

void receive(byte[] data, String ip, int port)
{
  String message = new String(data);
  processMessage(message, ip, port);
}

void processMessage(String message, String ip, int port)
{
  String[] message_list = split(message, ",");
  if (message_list[0].equals("HUMAN_COMMAND"))
  {
    rob_vel.x = float(message_list[1]);
    rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      human_robot.updatePosition(rob_vel);
    }
  }
  if (message_list[0].equals("AUTONOMY_COMMAND"))
  {
    rob_vel.x = float(message_list[1]);
    rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      autonomy_robot.updatePosition(rob_vel);
    }
  }
  if (message_list[0].equals("GOALPOS"))
  {
    int num_goals = int(message_list[1]);
    assert(message_list.length - 2 == (2*num_goals));
    init_goals(message_list);
    isGoalInitialized = true;
    
  }
}

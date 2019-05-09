void sendUDP(String message, String ip, int port, UDP udp)
{
  //byte[] data = message.getBytes();
  udp.send(message, ip, port);
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
    human_rob_vel.x = float(message_list[1]);
    human_rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      human_robot.updatePosition(human_rob_vel);
    }
  }
  if (message_list[0].equals("AUTONOMY_COMMAND"))
  {
    autonomy_rob_vel.x = float(message_list[1]);
    autonomy_rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      autonomy_robot.updatePosition(autonomy_rob_vel);
    }
  }
  if (message_list[0].equals("AUTONOMY_GOALPOS"))
  {
    int num_goals = int(message_list[1]);
    assert(message_list.length - 2 == (2*num_goals));
    init_autonomy_goals(message_list);
    isAutonomyGoalInitialized = true;
  }
  
  if(message_list[0].equals("AUTONOMY_ROBOTPOS"))
  {
    init_autonomy_robot(message_list);
    isAutonomyRobotInitialized = true;
  }
  
  if (message_list[0].equals("HUMAN_GOALPOS"))
  {
    int num_goals = int(message_list[1]);
    assert(message_list.length - 2 == (2*num_goals));
    init_human_goals(message_list);
    isHumanGoalInitialized = true;
  }
  
  if(message_list[0].equals("HUMAN_ROBOTPOS"))
  {
    init_human_robot(message_list);
    isHumanRobotInitialized = true;
  }
}

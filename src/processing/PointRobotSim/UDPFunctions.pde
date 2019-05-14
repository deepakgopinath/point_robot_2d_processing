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
  //CONTROL COMMAND FOR HUMAN CONTROLLED ROBOT
  if (message_list[0].equals("H_COMMAND"))
  {
    h_r_vel.x = float(message_list[1]);
    h_r_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      h_r.updatePosition(h_r_vel);
    }
  }
  //CONTROL COMMAND FOR AUTONOMY CONTROLLED ROBOT
  if (message_list[0].equals("A_COMMAND"))
  {
    a_r_vel.x = float(message_list[1]);
    a_r_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      a_r.updatePosition(a_r_vel);
    }
  }

  //AUTONOMY GOAL POSE INITIALIZATION
  if (message_list[0].equals("AUTONOMY_GOALPOS"))
  {
    int num_goals = int(message_list[1]);
    assert(message_list.length - 2 == (2*num_goals));
    init_a_g(message_list);
    isAGInitialized = true;
  }

  //AUTONOMY ROBOT POS INITIALIZATION

  if (message_list[0].equals("AUTONOMY_ROBOTPOS"))
  {  
    init_a_r(message_list);
    isARInitialized = true;
  }

  //HUMAN GOAL POSE INITIALIZATION
  if (message_list[0].equals("HUMAN_GOALPOS"))
  {

    int num_goals = int(message_list[1]);
    assert(message_list.length - 2 == (2*num_goals));
    init_h_g(message_list);
    isHGInitialized = true;
  }

  //HUMAN ROBOT POSE INITIALIZATION

  if (message_list[0].equals("HUMAN_ROBOTPOS"))
  {
    init_h_r(message_list);
    isHRInitialized = true;
  }
}

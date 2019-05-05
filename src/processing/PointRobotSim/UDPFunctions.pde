void sendUDP(String message, String ip, int port, UDP udp)
{
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
    println("X:", rob_vel.x);
    println("Y:", rob_vel.y);
    rob_vel.x = float(message_list[1]);
    rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      human_robot.updatePosition(rob_vel);
    }
  }
  if (message_list[0].equals("AUTONOMY_COMMAND"))
  {
    println("X:", rob_vel.x);
    println("Y:", rob_vel.y);
    rob_vel.x = float(message_list[1]);
    rob_vel.y = float(message_list[2]);
    if (allInitialized)
    {
      autonomy_robot.updatePosition(rob_vel);
    }
  }
}

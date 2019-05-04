void sendUDP(String message, String ip, int port, UDP udp)
{
  udp.send(message, ip, port);
}

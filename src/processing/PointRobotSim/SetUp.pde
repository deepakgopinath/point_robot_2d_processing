void init_a_g(String[] message_list)
{ 
  int num_goals = int(message_list[1]);
  float a_g_x, a_g_y;
  a_gList.clear();
  for (int i=0; i < num_goals; i++)
  {
    a_g_x = float(message_list[2*(i+1)]);
    a_g_y = float(message_list[2*(i+1) + 1]);

    a_gList.add(new Goal(a_g_x, a_g_y, GOAL_RADIUS, goal_shapes.get(i)));
  }
  isAGInitialized = true;
}

void init_a_r(String[] message_list)
{
  PVector ar_pos = new PVector(float(message_list[1]), float(message_list[2]));
  a_r.setPosition(ar_pos);
  isARInitialized = true;
}

//Human

void init_h_g(String[] message_list)
{
  int num_goals = int(message_list[1]);
  float h_g_x, h_g_y;
  h_gList.clear();
  for (int i=0; i < num_goals; i++)
  {
    h_g_x = float(message_list[2*(i+1)]);
    h_g_y = float(message_list[2*(i+1) + 1]);

    h_gList.add(new Goal(h_g_x, h_g_y, GOAL_RADIUS, goal_shapes.get(i)));
  }
  isHGInitialized = true;
}

void init_h_r(String[] message_list)
{
  PVector hr_pos = new PVector(float(message_list[1]) + width/2.0, float(message_list[2]));
  h_r.setPosition(hr_pos);
  isHRInitialized = true;
}

void instantiateRobots()
{
  a_r = new Robot(0.0, 0.0, ROBOT_RADIUS, A_R_LB, A_R_RB, a_r_color);
  h_r = new Robot(0.0, 0.0, ROBOT_RADIUS, H_R_LB, H_R_RB, h_r_color);

  a_r_pos = a_r.getPosition();
  h_r_pos = h_r.getPosition();

  a_r_vel = a_r.getVelocity();
  h_r_vel = h_r.getVelocity();
}

void instantiateGoalLists()
{
  a_gList = new ArrayList<Goal>();
  h_gList = new ArrayList<Goal>();
}

void instantiateUDP()
{
  a_udp = new UDP(this, A_HOST_PORT);
  a_udp.listen(true);
  h_udp = new UDP(this, H_HOST_PORT);
  h_udp.listen(true);
  key_udp = new UDP(this, KEY_HOST_PORT);
  key_udp.listen(true);
}

void setUpDicts()
{
  goal_color.set("Triangle", "r");
  goal_color.set("Square", "g");

  goal_shapes.put(0, "Triangle");
  goal_shapes.put(1, "Square");
}

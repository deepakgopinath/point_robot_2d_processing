void init_autonomy_goals(String[] message_list)
{
  int num_goals = int(message_list[1]);
  String autonomy_goal_color = "r";
  float goal_autonomy_x, goal_autonomy_y;

  for (int i=0; i < num_goals; i++)
  {
    goal_autonomy_x = float(message_list[2*(i+1)]);
    goal_autonomy_y = float(message_list[2*(i+1) + 1]);

    autonomy_goalList.add(new Goal(goal_autonomy_x, goal_autonomy_y, GOAL_RADIUS, autonomy_goal_color));
  }
  isAutonomyGoalInitialized = true;
}

void init_autonomy_robot(String[] message_list)
{
  PVector ar_pos = new PVector(float(message_list[1]), float(message_list[2]));
  autonomy_robot.setPosition(ar_pos);
  isAutonomyRobotInitialized = true;
}

void init_human_goals(String[] message_list)
{
  int num_goals = int(message_list[1]);
  String autonomy_goal_color = "r";
  float goal_autonomy_x, goal_autonomy_y;
}

void init_human_robot(String[] message_list)
{
}

void instantiateRobots()
{
  autonomy_robot = new Robot(0.0, 0.0, ROBOT_RADIUS, AUTONOMY_ROBOT_LB, AUTONOMY_ROBOT_RB, autonomy_rob_color);
  human_robot = new Robot(0.0, 0.0, ROBOT_RADIUS, HUMAN_ROBOT_LB, HUMAN_ROBOT_RB, human_rob_color);

  autonomy_rob_pos = autonomy_robot.getPosition();
  human_rob_pos = human_robot.getPosition();

  autonomy_rob_vel = autonomy_robot.getVelocity();
  human_rob_vel = human_robot.getVelocity();
}

void instantiateGoalLists()
{
  autonomy_goalList = new ArrayList<Goal>();
  human_goalList = new ArrayList<Goal>();
  goal_color.set("Triangle", "r");
  goal_color.set("Square", "g");
}

void instantiateUDP()
{
  autonomy_udp = new UDP(this, AUTONOMY_HOST_PORT);
  autonomy_udp.listen(true);
  human_udp = new UDP(this, HUMAN_HOST_PORT);
  human_udp.listen(true);
  key_udp = new UDP(this, KEY_HOST_PORT);
  key_udp.listen(true);  
}

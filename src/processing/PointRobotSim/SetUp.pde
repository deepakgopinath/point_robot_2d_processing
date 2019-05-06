void init_goals(String[] message_list)
{
  int num_goals = int(message_list[1]);
  String autonomy_goal_color = "r";
  String human_goal_color = "g";
  float goal_autonomy_x, goal_autonomy_y, goal_human_x, goal_human_y;

  for (int i=0; i < num_goals; i++)
  {
    goal_autonomy_x = float(message_list[2*(i+1)]);
    goal_autonomy_y = float(message_list[2*(i+1) + 1]);

    autonomy_goalList.add(new Goal(goal_autonomy_x, goal_autonomy_y, GOAL_RADIUS, autonomy_goal_color));

    goal_human_x = float(message_list[2*(i+1)]) + width/2.0;
    goal_human_y = float(message_list[2*(i+1) + 1]);
    
    human_goalList.add(new Goal(goal_human_x, goal_human_y, GOAL_RADIUS, human_goal_color));

  }
}

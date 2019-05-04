void displayGoals()
{
  for (Goal g : robot_goalList)
  {
    g.display();
  }
  for (Goal g: human_goalList)
  {
    g.display();
  }
}

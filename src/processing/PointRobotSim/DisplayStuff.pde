void displayGoals()
{
  for (Goal g : autonomy_goalList)
  {
    g.display();
  }
  for (Goal g: human_goalList)
  {
    g.display();
  }
}

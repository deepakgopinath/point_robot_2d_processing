void displayGoals()
{
  for (Goal g : autonomy_goalList)
  {
    g.display();
  }
  for (Goal g : human_goalList)
  {
    g.display();
  }
}

void displayRobots()
{
  autonomy_robot.display();
  human_robot.display();
}
void printText()
{
  textSize(TEXT_SIZE);
  textAlign(CENTER);
  fill(255, 0, 0);
  text("Autonomy Controlled", width/4.0, 30);
  fill(0, 255, 0);
  text("User Controlled", width/4.0 + width/2.0, 30);
}

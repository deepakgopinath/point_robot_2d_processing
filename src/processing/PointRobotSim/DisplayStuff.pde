void displayGoals()
{
  for (Goal g : a_gList)
  {
    g.display();
  }
  for (Goal g : h_gList)
  {
    g.display();
  }
}

void displayRobots()
{
  a_r.display();
  h_r.display();
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

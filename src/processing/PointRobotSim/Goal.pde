class Goal
{
  PVector position;
  float radius;
  color col;
  
  Goal(float init_x_, float init_y_, float init_radius_, String colorType)
  {
    position = new PVector(init_x_, init_y_);
    radius = init_radius_;
    col = color(255,255, 255);
    if (colorType == "r")
    {
      col = color(255, 0, 0);
    }else if (colorType == "g")
    {
      col = color(0, 255, 0);
    }
    else if (colorType == "b")
    {
      col = color(0, 0, 255);
    }
  }
  
  void display()
  {
    fill(col); //figrue a way to have color as a field variable
    stroke(0);
    ellipseMode(RADIUS);
    ellipse(position.x, position.y, radius, radius);
  }
  
  PVector getPosition()
  {
    return position;
  }
}

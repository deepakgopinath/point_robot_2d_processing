class Goal
{
  PVector position;
  float radius;
  color col;
  String shape;
   
  Goal(float init_x_, float init_y_, float init_radius_, String shape_)
  {
    position = new PVector(init_x_, init_y_);
    radius = init_radius_;
    shape = shape_;
    String colorType = goal_color.get(shape);
    col = color(255, 255, 255);
    if (colorType == "r")
    {
      col = color(255, 0, 0);
    } else if (colorType == "g")
    {
      col = color(0, 255, 0);
    } else if (colorType == "b")
    {
      col = color(0, 0, 255);
    }
  }

  void display()
  {
    fill(col); 
    stroke(0);
    if (shape.equals("Triangle"))
    { 
      triangle(position.x, position.y, position.x+radius, position.y, position.x, position.y-radius);
      ellipseMode(RADIUS);
      ellipse(position.x, position.y, 5, 5);
    }
    if (shape.equals("Square"))
    {
      rectMode(RADIUS);
      square(position.x, position.y, radius);
      ellipseMode(RADIUS);
      ellipse(position.x, position.y, 5, 5);
    }
  }

  PVector getPosition()
  {
    return position;
  }
}

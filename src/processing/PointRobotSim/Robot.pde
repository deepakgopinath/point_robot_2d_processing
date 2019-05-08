class Robot{
  PVector position;
  PVector velocity;
  float radius;
  float lb; // left bound
  float rb; // right bound
  color col;
  
  Robot(float init_x_, float init_y_, float init_radius_, float lb_, float rb_,  String colorType)
  {
    position = new PVector(init_x_, init_y_);
    radius = init_radius_;
    velocity = new PVector(0.0, 0.0);
    lb = lb_;
    rb = rb_;
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
    }else if (colorType == "w")
    {
      col = color(255);
    }else if(colorType == "gray")
    {
      col = color(127);
    }
  }
  
  void display()
  {
    fill(col);
    stroke(0);
    ellipseMode(RADIUS);
    ellipse(position.x, position.y, radius, radius);
  }
    
  //getters
  PVector getPosition()
  {
    return position;
  }
  void setPosition(PVector pos)
  {
    position.x = pos.x;
    position.y = pos.y;
  }
  PVector getVelocity()
  {
    return velocity;
  }
  
  //utility functions
  void updatePosition(PVector velocity)
  {
    position.add(velocity);
    checkBounds();
  }
  
  void checkBounds()
  {
      if (position.x < lb + radius || position.x > rb - radius)
      {
        if (position.x < lb + radius)
        {
          position.x = lb + radius;
        }else
        {
          position.x = rb - radius;
        }
      }
      
      if (position.y < radius  || position.y > height-radius)
      {
        if (position.y < radius)
        {
          position.y = 0 + radius;
        }
        else
        {
          position.y = height-radius;
        }
      }
  }
  
  
  
  
  
  
  
}

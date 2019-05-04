class Robot{
  PVector position;
  PVector velocity;
  float radius;
  float lb; // left bound
  float rb; // right bound
  color col;
  
  Robot(float init_x_, float init_y_, float init_radius_, float lb_, float rb_)
  {
    position = new PVector(init_x_, init_y_);
    radius = init_radius_;
    velocity = new PVector(0.0, 0.0);
    lb = lb_;
    rb = rb_;
    col = color(255, 255, 255);
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

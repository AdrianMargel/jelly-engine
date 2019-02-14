//simple vector class
public class Vector{
  float x;
  float y;
  public Vector(float x,float y){
    this.x=x;
    this.y=y;
  }
  public Vector(Vector v){
    x=v.x;
    y=v.y;
  }
  public void addVec(Vector v){
    x+=v.x;
    y+=v.y;
  }
  public void subVec(Vector v){
    x-=v.x;
    y-=v.y;
  }
  public float getAngle(){
    return (float)Math.atan2(y,x);
  }
  public float getMagnitude(){
    return (float)Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
  }
}

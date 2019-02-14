/*
  Jelly Engine
  ------------
  A simple softbody physics engine using shape memory.
  Click to have the shape deform and bounce around!
  
  written by Adrian Margel, Spring 2018
*/

//a connection between two nodes
class Connection {
  //the two connected nodes
  private Node c1;
  private Node c2;
  
  //how much of an offset the node's connections have from the sorted arraylist
  public int off;
  
  //strength of connection
  public float strength;
  public float resist;
  //how long the connection is
  private float distance;
  
  public Connection(float tdist, Node tc1, Node tc2) {
    resist=0.5;
    strength=0.5;
    distance=tdist;
    c1=tc1;
    c2=tc2;
  }
  
  //applies torque to the connection
  public void turn(Vector turn,Node requester){
    stroke(255,0,0);
    strokeWeight(10);
    if(c1==requester){
      c2.velo2.addVec(turn);
    }else if(c2==requester){
      c1.velo2.addVec(turn);
    }
  }
  
  //apply force to pull nodes together if they are far away or push them apart if they are too close
  public void calcPull() {
    //calculate difference between the distance of the connection and the distance between the nodes
    float diff=(float)Math.sqrt(Math.pow(c2.pos.x-c1.pos.x,2)+Math.pow(c2.pos.y-c1.pos.y,2))-(distance);
    
    if (diff!=0) {
      //apply force to nodes
      
      //calculate the distance between where the points will be next frame
      float tempDist=(float)Math.sqrt(Math.pow((c2.pos.x+c2.velo.x)-(c1.pos.x+c1.velo.x),2)+Math.pow((c2.pos.y+c2.velo.y)-(c1.pos.y+c1.velo.y),2));
      //calculate (normalized distance between where the points will be) * (difference between distance) * strength
      float tempx=(float)(((c2.pos.x+c2.velo.x)-(c1.pos.x+c1.velo.x))/tempDist*diff*strength);
      float tempy=(float)(((c2.pos.y+c2.velo.y)-(c1.pos.y+c1.velo.y))/tempDist*diff*strength);
      
      //apply the force divided by the total number of connections the node has
      //connection point 1
      c1.velo2.x+=tempx/(float)c1.cons;
      c1.velo2.y+=tempy/(float)c1.cons;
      //connection point 2
      c2.velo2.x+=-tempx/(float)c2.cons;
      c2.velo2.y+=-tempy/(float)c2.cons;
    }
  }
  
  //dissipate some of the energy between the two nodes
  public void calcResist() {
    //calc connection angle
    float cAngle=(float)Math.atan2(c1.pos.y-c2.pos.y, c1.pos.x-c2.pos.x);
    //calc angle and speed of nodes
    float angle1=(float)Math.atan2(c1.velo.y, c1.velo.x);
    float speed1=(float)Math.sqrt(Math.pow(c1.velo.y,2)+Math.pow(c1.velo.x,2));
    float angle2=(float)Math.atan2(c2.velo.y, c2.velo.x);
    float speed2=(float)Math.sqrt(Math.pow(c2.velo.y,2)+Math.pow(c2.velo.x,2));
    //rotate node angles
    angle1-=cAngle;
    angle2-=cAngle;
    //calculate sin and cos of rotated nodes
    float s1=(float)Math.sin(angle1)*speed1;
    float cs1=(float)Math.cos(angle1)*speed1;
    float s2=(float)Math.sin(angle2)*speed2;
    float cs2=(float)Math.cos(angle2)*speed2;
    //calculate speed average
    float sa=(cs1+cs2)/2;
    //subtract speed differences off of cos
    cs1-=sa;
    cs2-=sa;
    //apply resistance
    cs1*=resist;
    cs2*=resist;
    //add speed average back on
    cs1+=sa;
    cs2+=sa;
    //calculate new angles
    angle1=(float)Math.atan2(s1, cs1);
    speed1=(float)Math.sqrt(Math.pow(s1,2)+Math.pow(cs1,2));
    angle2=(float)Math.atan2(s2, cs2);
    speed2=(float)Math.sqrt(Math.pow(s2,2)+Math.pow(cs2,2));
    //rotate new angles
    angle1+=cAngle;
    angle2+=cAngle;
    //update speed
    c1.velo2.y+=(Math.sin(angle1)*speed1-c1.velo.y)/(float)c1.cons;
    c1.velo2.x+=(Math.cos(angle1)*speed1-c1.velo.x)/(float)c1.cons;
    c2.velo2.y+=(Math.sin(angle2)*speed2-c2.velo.y)/(float)c2.cons;
    c2.velo2.x+=(Math.cos(angle2)*speed2-c2.velo.x)/(float)c2.cons;
  }
  
  //returns a positive angle betweeen c1 and c2 from the perspective of c1 or c2
  public float getPosAngle(Node requester){
    if(c1==requester){
      Vector temp=new Vector(c1.pos);
      temp.subVec(c2.pos);
      float angle=temp.getAngle();
      if(angle<0){
        angle+=(float)Math.PI*2f;
      }
      return angle;
    }else if(c2==requester){
      Vector temp=new Vector(c2.pos);
      temp.subVec(c1.pos);
      float angle=temp.getAngle();
      if(angle<0){
        angle+=(float)Math.PI*2f;
      }
      return angle;
    }else{
      return -1;
    }
  }
}


class Node {
  //position
  public Vector pos;
  //velocity
  public Vector velo;
  //velocity buffer
  public Vector velo2;
  
  //number of connections
  public int cons;
  //all connections
  ArrayList<Connection> sorts=new ArrayList<Connection>();

  public Node(float tx, float ty) {
    pos=new Vector(tx,ty);
    velo=new Vector(0,0);
    velo2=new Vector(0,0);
  }
  
  //add new connection
  public void addCon(Connection addCon){
    sorts.add(addCon);
    bubbleSort(sorts);
    cons=sorts.size();
  }
  
  //flip velocity buffer
  public void flip(){
    velo.addVec(velo2);
    velo2=new Vector(0,0);
  }
  
  //move position based on velocity
  public void move() {
    pos.addVec(velo);
  }
  
  //allow connections to "collide"
  private void selfCollideLocal() {
    //uses shape memory
    
    //sort arraylist
    float fixS=1;
    ArrayList<Connection> compareList=new ArrayList<Connection>();
    for (int i=0; i<sorts.size(); i++) {
      compareList.add(sorts.get(i));
    }
    bubbleSort(compareList);

    //compare
    //calculate closest offset to the sorted array
    int offsetTotal=0;
    int size=sorts.size();
    float half=(float)size/2;
    for (int i=0; i<sorts.size(); i++) {
      for (int j=0; j<compareList.size(); j++) {
        if (sorts.get(i)==compareList.get(j)) {
          int offset=i-j;
          if (offset<-half) {
            offset+=sorts.size();
          } else if (offset>half) {
            offset-=sorts.size();
          }
          sorts.get(i).off=offset;
          offsetTotal+=abs(offset);
        }
      }
    }
    int finalOffset=lowestOffset(offsetTotal);

    //apply force to connections to get them closer to the sorted order
    for (int i=0; i<sorts.size(); i++) {
      int diff=sorts.get(i).off-finalOffset;
      if (diff<-half) {
        diff+=size;
      } else if (diff>half) {
        diff-=size;
      }
      if (diff<0) {
        float turnAngle=sorts.get(i).getPosAngle(this)+PI/2;
        Vector turn=new Vector(cos(turnAngle)*fixS,sin(turnAngle)*fixS);
        sorts.get(i).turn(turn,this);
      } else if (diff>0) {
        float turnAngle=sorts.get(i).getPosAngle(this)-PI/2;
        Vector turn=new Vector(cos(turnAngle)*fixS,sin(turnAngle)*fixS);
        sorts.get(i).turn(turn,this);
      }
    }
  }

  //calculates an offset that will result in the unsorted connections moving the shortest distance to reach their sorted counterparts
  private int lowestOffset(int firstSize) {
    int lowestOff=firstSize;
    int lowestId=0;//the first size should always be from a 0 id
    int size=sorts.size();
    float half=(float)size/2;
    for (int i=1; i<sorts.size(); i+=1) {//0 has been tested by first id so start at 1 (small optimisation)
      int compareOff=testOffset(i);
      if (compareOff<lowestOff) {
        lowestOff=compareOff;
        lowestId=i;
      }
    }

    if (lowestId<-half) {
      lowestId+=size;
    }
    if (lowestId>half) {
      lowestId-=size;
    }
    return lowestId;
  }

  //tests a given offset value and sees how much the connections would have to move to reach it
  private int testOffset(int offset) {
    int total=0;
    int size=sorts.size();
    float half=(float)size/2;
    for (Connection a : sorts) {
      int off=a.off-offset;
      if (off<-half) {
        off+=size;
      }
      if (off>half) {
        off-=size;
      }

      total+=abs(off);
    }
    return total;
  }

  //simple sorting algorithm to figure out the order the connections should be in
  private void bubbleSort(ArrayList<Connection> arr) {  
    int n = arr.size();
    Connection temp = null;
    for (int i=0; i < n; i++) {
      for (int j=1; j < (n-i); j++) {
        if (arr.get(j-1).getPosAngle(this) > arr.get(j).getPosAngle(this)) {
          temp = arr.get(j-1);
          arr.set(j-1, arr.get(j));
          arr.set(j, temp);
        }
      }
    }
  }
}

//Soft Body
class SBody {
  //the nodes it is made up of
  ArrayList<Node> nodes=new ArrayList<Node>();
  //the connections between it's nodes
  ArrayList<Connection> connects=new ArrayList<Connection>();

  public SBody() {
    //spawn starting shape
    //add nodes
    for (int i=0; i<5; i++) {
      for (int j=0; j<15; j++) {
        nodes.add(new Node((i+1)*100+25*(j%2), (j+1)*100));
      }
    }
    //connect nodes
    for (int i=0; i<nodes.size(); i++) {
      for (int j=0; j<nodes.size(); j++) {
        if (Math.sqrt(Math.pow(nodes.get(i).pos.x-nodes.get(j).pos.x, 2)+Math.pow(nodes.get(i).pos.y-nodes.get(j).pos.y, 2))<=150&&i!=j) {
          connects.add(new Connection((float)Math.sqrt(Math.pow(nodes.get(i).pos.x-nodes.get(j).pos.x, 2)+Math.pow(nodes.get(i).pos.y-nodes.get(j).pos.y, 2)), nodes.get(i), nodes.get(j)));
        }
      }
    }
    
    //make nodes aware of connections
    update();
  }

  //update the connections to make the nodes aware of them
  private void update() {
    for (int i=0; i<connects.size(); i++) {
      connects.get(i).c1.addCon(connects.get(i));
      connects.get(i).c2.addCon(connects.get(i));
    }
  }
  
  //simulate the shape
  public void step() {
    for (int i=0; i<1; i++) {
      for (int j=0; j<1; j++) {
      calcPull();
      flip();
      }
      for (int j=0; j<1; j++) {
      calcResist();
      flip();
      }
      for (int j=0; j<1; j++) {
      selfCollide();
      flip();
      }
      move();
    }
  }
  
  //flip node velocity buffers
  private void flip(){
    for(Node n:nodes){
      n.flip();
    }
  }
  //calculate pull on all nodes
  private void calcPull() {
    for(Connection c:connects){
      c.calcPull();
    }
  }
  //calculate resistance on all nodes
  private void calcResist() {
    for(Connection c:connects){
      c.calcResist();
    }
  }
  //move all nodes
  private void move() {
    for(Node n:nodes){
      n.move();
    }
  }
  //have shape collide with itself
  private void selfCollide() {
    //use shape memory to handle small issues
    selfCollideLocal();
    //currently unused method to do global self-collision
    //selfCollideGlobal();
  }
  //apply shape memory
  private void selfCollideLocal() {
    //uses keychain approach
    for (Node n : nodes) {
      n.selfCollideLocal();
    }
  }
  /*private void selfColideGlobal() {
    //uses global tests to see if points lie inside or outside of the polygon
  }*/
  //display the shape
  void display(){
    stroke(0);
    strokeWeight(2);
    for(Connection c:connects){
      line(c.c1.pos.x*zoom,c.c1.pos.y*zoom,c.c2.pos.x*zoom,c.c2.pos.y*zoom);
    }
  }
}

//create a test soft body
SBody test = new SBody();
//camera zoom
float zoom=0.5;

void setup(){
  //init soft body
  test=new SBody();
  //set window size
  size(800,800);
}

void draw(){
  background(255);
  //run the soft body
  test.step();
  //display the soft body
  test.display();
  
  //if mouse is down bring the first node in the soft body to the mouse
  if (mouseDown) {
    test.nodes.get(0).velo.x=mouseX/zoom-test.nodes.get(0).pos.x;
    test.nodes.get(0).velo.y=mouseY/zoom-test.nodes.get(0).pos.y;
  }
}

//handle mouse inputs
boolean mouseDown=false;
void mousePressed(){
  mouseDown=true;
}
void mouseReleased(){
  mouseDown=false;
}

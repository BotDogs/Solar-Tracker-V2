#include <math.h>
#include <AccelStepper.h>
#include <math.h>
//unit of the codes will be mm and ms
const int num = 2; //number of control points on the solar panels
double TPC = 100.0; //Time Per cycle ms, how often the system recalculates the speed of the motors, smaller means finer speed adjustments
double MaxA = (M_PI)/1000.0*TPC; //maximum angle of rotation per cycle, input angle per second in bracket
double Ix[num], Iy[num], Iz[num]; //Initial control point
double Cx[num], Cy[num], Cz[num]; //current location of control point
double Nx[num], Ny[num], Nz[num]; //Next point location
double Tx[num], Ty[num], Tz[num]; //final target point-currently unused
double CVx, CVy, CVz; //current direction vector
double TVx, TVy, TVz; //target direction vector- will be taken from the solar tracker formula
double RVx, RVy, RVz; //rotation vector
double Qw[num], Qx[num], Qy[num], Qz[num]; //quaternion mid product- can be phased out later upon confirmation of the working quaternion multiplication
double QIw, QIx, QIy, QIz; //quaternion mid product- can be phased out later upon confirmation of the working quaternion multiplication
double RA=0, AA=0; //RA=the angle that the panel will move in a cycle, AA = the angle between the target and current direction vector
double CPx[num], CPy[num], CPz[num]; //mounting point of the piston/actuating point of the wire
double SPD[num]; //the speed and direciton of the wire/piston, mm/ms
float  spoolradius = 4.75;
float  solarradius = 80.0;
float  baseradius = 53.5;
int dir1=1,dir2=1;
int ta=-1;
const int digits = 2; // how many comma separated fields we expect
int Ind = 0;            // the current field being received
const int maxta=6;
float values[maxta][2] = {{90,60}, {180,45}, {0,75}, {270,60}, {0,90}, {130,45}};   // array holding values for all the fields
int dp=0;

int count=0;
int count1=0;


AccelStepper S1(AccelStepper::FULL4WIRE, 2, 3, 4, 5);
AccelStepper S2(AccelStepper::FULL4WIRE, 6, 7, 8, 9);
//----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  S1.setMaxSpeed(500);
  S1.setAcceleration(1000);
  S2.setMaxSpeed(500);
  S2.setAcceleration(1000);
  while(!Serial.available());
  vals();
  initiate();
  Speed();
}
//----------------------------------------------------------------------
void vals() {
  Cx[0] = solarradius * cos(7 * M_PI / 4);Ix[0]=Cx[0];Nx[0]=Cx[0];
  Cy[0] = solarradius * sin(7 * M_PI / 4),Iy[0]=Cy[0],Ny[0]=Cy[0];
  Cz[0] = 0,Iz[0]=Cz[0],Nz[0]=Cz[0];
  Cx[1] = solarradius * cos(M_PI / 4),Ix[1]=Cx[1],Nx[1]=Cx[1];
  Cy[1] = solarradius * sin(M_PI / 4),Iy[1]=Cy[1],Ny[1]=Cy[1];
  Cz[1] = 0,Iz[1]=Cz[1],Nz[1]=Cz[1];
  CPx[0] = baseradius * cos(7 * M_PI / 4);
  CPy[0] = baseradius * sin(7 * M_PI / 4);
  CPz[0] = -67.5;
  CPx[1] = baseradius * cos(M_PI / 4);
  CPy[1] = baseradius * sin(M_PI / 4);
  CPz[1] = -67.5;
  CVx = 0, CVy = 0, CVz = 1;
  TVx = 0, TVy = 0, TVz = 1;
  SPD[0]=0;
  SPD[1]=0;
}
//-----------------------------------------------------------------------
void Speed() {
  //finds the change in distance between the control point and point mounted to the panel
  /*for (int i = 0; i < num; i++) {
    //closed loop control
    SPD[i] = (pow(pow(Nx[i] - CPx[i], 2) + pow(Ny[i] - CPy[i], 2) + pow(Nz[i] - CPz[i], 2), .5) - pow(pow(Cx[i] - CPx[i], 2) + pow(Cy[i] - CPy[i], 2) + pow(Cz[i] - CPz[i], 2), .5))/TPC/(2*M_PI*spoolradius)*(200/(2*M_PI));
    //resulting speed is in mm/ms 
  }*/
  int i=0;
                            //distance between the control point and the next point                   distance between control point and intial point at 90 deg                             distance of wire on the spool                                                                           
  SPD[i]=dir1*(pow(pow(Nx[i] - CPx[i], 2) + pow(Ny[i] - CPy[i], 2) + pow(Nz[i] - CPz[i], 2), .5) - (pow(pow(Ix[i] - CPx[i], 2) + pow(Iy[i] - CPy[i], 2) + pow(Iz[i] - CPz[i], 2), .5) - dir1*spoolradius*static_cast<float>(S1.currentPosition())*M_PI/100.0))*1000.0/TPC/(2*M_PI*spoolradius)*(200.0/(2*M_PI));
    //distance calculated using motor position
  i=1;
  SPD[i]=dir2*(pow(pow(Nx[i] - CPx[i], 2) + pow(Ny[i] - CPy[i], 2) + pow(Nz[i] - CPz[i], 2), .5) - (pow(pow(Ix[i] - CPx[i], 2) + pow(Iy[i] - CPy[i], 2) + pow(Iz[i] - CPz[i], 2), .5) - dir2*spoolradius*static_cast<float>(S2.currentPosition())*M_PI/100.0))*1000.0/TPC/(2*M_PI*spoolradius)*(200.0/(2*M_PI));
  if(isnan(SPD[0]) || (SPD[0]<1&&SPD[0]>-1)){
    S1.setSpeed(0);
  }else{
    S1.setSpeed(-SPD[0]);
  }
  if(isnan(SPD[1]) || (SPD[1]<1&&SPD[1]>-1)){
    S2.setSpeed(0);
  }else{
    S2.setSpeed(-SPD[1]);
  }
}
//-----------------------------------------------------------------------
void updater() {
  //cross product to find rotation vector
  RVx = CVy * TVz - CVz * TVy;
  RVy = CVz * TVx - CVx * TVz;
  RVz = CVx * TVy - CVy * TVx;
  
  //calculating the angle between current and target direction vector
  AA = asin(pow(pow(RVx, 2) + pow(RVy, 2) + pow(RVz, 2), .5) / (pow(pow(TVx, 2) + pow(TVy, 2) + pow(TVz, 2), .5) * pow(pow(CVx, 2) + pow(CVy, 2) + pow(CVz, 2), .5)));
  float mag=pow(pow(RVx, 2) + pow(RVy, 2) + pow(RVz, 2), .5);
  RVx = RVx/mag;
  RVy = RVy/mag;
  RVz = RVz/mag;
  
  for(int i=0;i<num;i++){
      Cx[i]=Nx[i];
      Cy[i]=Ny[i];
      Cz[i]=Nz[i];
      
    }
  //restricts the maximum rotation angle per second to MaxA
  if(!isnan(AA)&&!static_cast<int>(AA*10000)==0)
  {
  if (AA > MaxA) {
    RA = MaxA;
  }
  else {
    RA = AA;
  }
  
  //calculating the next point that the machine will travel to
  for (int i = 0; i < num; i++) {
    Cx[i] = Nx[i];
    Cy[i] = Ny[i];
    Cz[i] = Nz[i];
    
    //first step of quaternion multiplication
    Qw[i] = cos(RA / 2) * 0     - sin(RA / 2) * RVx * Cx[i] - sin(RA / 2) * RVy * Cy[i] - sin(RA / 2) * RVz * Cz[i];
    Qx[i] = cos(RA / 2) * Cx[i] + sin(RA / 2) * RVx * 0     + sin(RA / 2) * RVy * Cz[i] - sin(RA / 2) * RVz * Cy[i];
    Qy[i] = cos(RA / 2) * Cy[i] - sin(RA / 2) * RVx * Cz[i] + sin(RA / 2) * RVy * 0     + sin(RA / 2) * RVz * Cx[i];
    Qz[i] = cos(RA / 2) * Cz[i] + sin(RA / 2) * RVx * Cy[i] - sin(RA / 2) * RVy * Cx[i] + sin(RA / 2) * RVz * 0;
    
    //second step of quaternion multiplication
    //Qw[i] = Qw[i] * cos(-RA / 2) - Qx[i] * sin(-RA / 2) * RVx - Qy[i] * sin(-RA / 2) * RVy - Qz[i] * sin(-RA / 2) * RVz;
    Nx[i] = Qw[i] * sin(-RA / 2) * RVx + Qx[i] * cos(-RA / 2)       + Qy[i] * sin(-RA / 2) * RVz - Qz[i] * sin(-RA / 2) * RVy;
    Ny[i] = Qw[i] * sin(-RA / 2) * RVy - Qx[i] * sin(-RA / 2) * RVz + Qy[i] * cos(-RA / 2)       + Qz[i] * sin(-RA / 2) * RVx;
    Nz[i] = Qw[i] * sin(-RA / 2) * RVz + Qx[i] * sin(-RA / 2) * RVy - Qy[i] * sin(-RA / 2) * RVx + Qz[i] * cos(-RA / 2);
    double mag=pow(pow(Nx[i],2)+pow(Ny[i],2)+pow(Ny[i],2),.5);
    Nx[i] = Nx[i]/mag*80.0;
    Ny[i] = Ny[i]/mag*80.0;
    Nz[i] = Nz[i]/mag*80.0;
    Serial.println(AA*180/M_PI);
    
  }
  //quaternion multiplication rotating the current direction vector
  QIw = cos(RA / 2) * 0   - sin(RA / 2) * RVx * CVx - sin(RA / 2) * RVy * CVy - sin(RA / 2) * RVz * CVz;
  QIx = cos(RA / 2) * CVx + sin(RA / 2) * RVx * 0   + sin(RA / 2) * RVy * CVz - sin(RA / 2) * RVz * CVy;
  QIy = cos(RA / 2) * CVy - sin(RA / 2) * RVx * CVz + sin(RA / 2) * RVy * 0   + sin(RA / 2) * RVz * CVx;
  QIz = cos(RA / 2) * CVz + sin(RA / 2) * RVx * CVy - sin(RA / 2) * RVy * CVx + sin(RA / 2) * RVz * 0;
  
  //second step of quaternion multiplication
  //Qw[i] = Qw[i] * cos(-RA / 2) - Qx[i] * sin(-RA / 2) * RVx - Qy[i] * sin(-RA / 2) * RVy - Qz[i] * sin(-RA / 2) * RVz; //unused component of our application of quaternion multiplication
  CVx = QIw * sin(-RA / 2) * RVx + QIx * cos(-RA / 2)       + QIy * sin(-RA / 2) * RVz - QIz * sin(-RA / 2) * RVy;
  CVy = QIw * sin(-RA / 2) * RVy - QIx * sin(-RA / 2) * RVz + QIy * cos(-RA / 2)       + QIz * sin(-RA / 2) * RVx;
  CVz = QIw * sin(-RA / 2) * RVz + QIx * sin(-RA / 2) * RVy - QIy * sin(-RA / 2) * RVx + QIz * cos(-RA / 2);
  //mag=pow(pow(CVx,2)+pow(CVy,2)+pow(CVz,2),.5);
  //CVx = CVx/mag;
  //CVy = CVy/mag;
  //CVz = CVz/mag;
  
 }
}
//-----------------------------------------------------------------------
void initiate() {
  //cross product to find rotation vector
  RVx = (CVy * TVz - CVz * TVy);
  RVy = (CVz * TVx - CVx * TVz);
  RVz = (CVx * TVy - CVy * TVx);
  //calculating the angle between current and target direction vector
  AA = asin(pow(pow(RVx, 2) + pow(RVy, 2) + pow(RVz, 2), .5) / (pow(pow(TVx, 2) + pow(TVy, 2) + pow(TVz, 2), .5) * pow(pow(CVx, 2) + pow(CVy, 2) + pow(CVz, 2), .5)));
  double mag=pow(pow(RVx, 2) + pow(RVy, 2) + pow(RVz, 2), .5);
  RVx = RVx/mag;
  RVy = RVy/mag;
  RVz = RVz/mag;
  //restricts the maximum rotation angle per second to MaxA
 if(!isnan(AA)&&!static_cast<int>(AA*10000)==0){
  if (AA > MaxA) {
    RA = MaxA;
  }
  else {
    RA = AA;
  }
  for (int i = 0; i < num; i++) {  
    //first step of quaternion multiplication
    Qw[i] = cos(RA / 2) * 0     - sin(RA / 2) * RVx * Cx[i] - sin(RA / 2) * RVy * Cy[i] - sin(RA / 2) * RVz * Cz[i];
    Qx[i] = cos(RA / 2) * Cx[i] + sin(RA / 2) * RVx * 0     + sin(RA / 2) * RVy * Cz[i] - sin(RA / 2) * RVz * Cy[i];
    Qy[i] = cos(RA / 2) * Cy[i] - sin(RA / 2) * RVx * Cz[i] + sin(RA / 2) * RVy * 0     + sin(RA / 2) * RVz * Cx[i];
    Qz[i] = cos(RA / 2) * Cz[i] + sin(RA / 2) * RVx * Cy[i] - sin(RA / 2) * RVy * Cx[i] + sin(RA / 2) * RVz * 0;
    //second step of quaternion multiplication
    //Qw[i] = Qw[i] * cos(-RA / 2) - Qx[i] * sin(-RA / 2) * RVx - Qy[i] * sin(-RA / 2) * RVy - Qz[i] * sin(-RA / 2) * RVz;
    Nx[i] = Qw[i] * sin(-RA / 2) * RVx + Qx[i] * cos(-RA / 2)       + Qy[i] * sin(-RA / 2) * RVz - Qz[i] * sin(-RA / 2) * RVy;
    Ny[i] = Qw[i] * sin(-RA / 2) * RVy - Qx[i] * sin(-RA / 2) * RVz + Qy[i] * cos(-RA / 2)       + Qz[i] * sin(-RA / 2) * RVx;
    Nz[i] = Qw[i] * sin(-RA / 2) * RVz + Qx[i] * sin(-RA / 2) * RVy - Qy[i] * sin(-RA / 2) * RVx + Qz[i] * cos(-RA / 2);
    
  }
 }
}
//-----------------------------------------------------------------------

void input(){
  if(SPD[0]<1&&SPD[0]>-1&&SPD[1]<1&&SPD[1]>-1){
    ta++;
    if(ta>=maxta){
      ta=0;
    }
    if(values[ta][1]<50){values[ta][1]=50;}
    if(values[ta][1]>140){values[ta][1]=140;}
    TVx = cos(values[ta][0]*M_PI/180.0)*cos(values[ta][1]*M_PI/180.0), TVy = sin(values[ta][0]*M_PI/180.0)*cos(values[ta][1]*M_PI/180.0), TVz = sin(values[ta][1]*M_PI/180.0);
    updater();
    Speed();
    count1++;
  }
}

void loop(){
  input();
  if(millis()/TPC>=count1){
    updater();
    Speed();
    count1++;
  }
  S1.runSpeed();
  S2.runSpeed();
  int i=0;
  Serial.println(ta);
  
}

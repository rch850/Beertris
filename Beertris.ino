//========================================================================
// TETRIS with M5STACK : 2018.01.20 Transplant by macsbug
// Controller : Buttons A = LEFT, B = RIGHT, C = START, ROTATE
// Display    : Left = 100x240, Center = 120x240, Right = 100x240
// Block      : 8ea, 12x12 pixel
// SD         : tetris.jpg : BackGround Image : R.G.B 320x240 pixel
// Github     : https://macsbug.wordpress.com/2018/01/20/tetris-with-m5stack/
//========================================================================
#include <M5Stack.h>                                       // M5STACK
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"

#define SerialDebug true  // Set to true to get Serial output for debugging

MPU9250 IMU;

uint16_t BlockImage[8][12][12];                            // Block
uint16_t backBuffer[240][120];                             // GAME AREA
const int Length = 12;     // the number of pixels for a side of a block
const int Width  = 10;     // the number of horizontal blocks
const int Height = 20;     // the number of vertical blocks
int screen[Width][Height] = {0}; //it shows color-numbers of all positions
struct Point {int X, Y;};
struct Block {Point square[4][4]; int numRotate, color;};
Point pos; Block block;
int rot, fall_cnt = 0;
bool started = false, gameover = false;
boolean but_A = false, but_LEFT = false, but_RIGHT = false;
int game_speed = 25; // 25msec
Block blocks[7] = {
  {{{{-1,0},{0,0},{1,0},{2,0}},{{0,-1},{0,0},{0,1},{0,2}},
  {{0,0},{0,0},{0,0},{0,0}},{{0,0},{0,0},{0,0},{0,0}}},2,1},
  {{{{0,-1},{1,-1},{0,0},{1,0}},{{0,0},{0,0},{0,0},{0,0}},
  {{0,0},{0,0},{0,0},{0,0}},{{0,0},{0,0},{0,0},{0,0}}},1,2},
  {{{{-1,-1},{-1,0},{0,0},{1,0}},{{-1,1},{0,1},{0,0},{0,-1}},
  {{-1,0},{0,0},{1,0},{1,1}},{{1,-1},{0,-1},{0,0},{0,1}}},4,3},
  {{{{-1,0},{0,0},{0,1},{1,1}},{{0,-1},{0,0},{-1,0},{-1,1}},
  {{0,0},{0,0},{0,0},{0,0}},{{0,0},{0,0},{0,0},{0,0}}},2,4},
  {{{{-1,0},{0,0},{1,0},{1,-1}},{{-1,-1},{0,-1},{0,0},{0,1}},
  {{-1,1},{-1,0},{0,0},{1,0}},{{0,-1},{0,0},{0,1},{1,1}}},4,5},
  {{{{-1,1},{0,1},{0,0},{1,0}},{{0,-1},{0,0},{1,0},{1,1}},
  {{0,0},{0,0},{0,0},{0,0}},{{0,0},{0,0},{0,0},{0,0}}},2,6},
  {{{{-1,0},{0,0},{1,0},{0,-1}},{{0,-1},{0,0},{0,1},{-1,0}},
  {{-1,0},{0,0},{1,0},{0,1}},{{0,-1},{0,0},{0,1},{1,0}}},4,7}
};
extern uint8_t tetris_img[];
//========================================================================
void setup(void) {
  M5.begin();                   // M5STACK INITIALIZE
  M5.Lcd.setBrightness(200);    // BRIGHTNESS = MAX 255
  M5.Lcd.fillScreen(BLACK);     // CLEAR SCREEN
  //----------------------------// Make Block ----------------------------
  make_block( 0, BLACK);        // Type No, Color
  make_block( 1, 0x00F0);       // DDDD     RED
  make_block( 2, 0xFBE4);       // DD,DD    PUPLE 
  make_block( 3, 0xFF00);       // D__,DDD  BLUE
  make_block( 4, 0xFF87);       // DD_,_DD  GREEN 
  make_block( 5, 0x87FF);       // __D,DDD  YELLO
  make_block( 6, 0xF00F);       // _DD,DD_  LIGHT GREEN
  make_block( 7, 0xF8FC);       // _D_,DDD  PINK
  //----------------------------------------------------------------------
  Wire.begin();
  // Start by performing self test and reporting values
  IMU.MPU9250SelfTest(IMU.SelfTest);
  Serial.print("x-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: acceleration trim within : ");
  Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
  Serial.print("x-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
  Serial.print("y-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
  Serial.print("z-axis self test: gyration trim within : ");
  Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

  // Calibrate gyro and accelerometers, load biases in bias registers
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

  IMU.initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  Serial.println("MPU9250 initialized for active data mode....");

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
  Serial.print(" I should be "); Serial.println(0x48, HEX);

  // Get magnetometer calibration from AK8963 ROM
  IMU.initAK8963(IMU.magCalibration);
  // Initialize device for active mode read of magnetometer
  Serial.println("AK8963 initialized for active data mode....");
  //----------------------------------------------------------------------
  // M5.Lcd.drawJpgFile(SD, "/tetris.jpg");     // Load background from SD
  M5.Lcd.drawJpg(tetris_img, 34215); // Load background from file data
  PutStartPos();                             // Start Position
  for (int i = 0; i < 4; ++i) screen[pos.X + 
   block.square[rot][i].X][pos.Y + block.square[rot][i].Y] = block.color;
  Draw();                                    // Draw block
}
//========================================================================
void loop() {
  if (gameover) return;

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0]*IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1]*IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2]*IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0]*IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1]*IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2]*IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0]*IMU.mRes*IMU.magCalibration[0] -
               IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1]*IMU.mRes*IMU.magCalibration[1] -
               IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2]*IMU.mRes*IMU.magCalibration[2] -
               IMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  IMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx*DEG_TO_RAD,
                         IMU.gy*DEG_TO_RAD, IMU.gz*DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
  IMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
  IMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                *(getQ()+2)));
  IMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
  IMU.pitch *= RAD_TO_DEG;
  IMU.yaw   *= RAD_TO_DEG;
  // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
  //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
  // - http://www.ngdc.noaa.gov/geomag-web/#declination
  IMU.yaw   -= 8.5;
  IMU.roll  *= RAD_TO_DEG;

  if(SerialDebug)
  {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(IMU.yaw, 2);
    Serial.print(", ");
    Serial.print(IMU.pitch, 2);
    Serial.print(", ");
    Serial.println(IMU.roll, 2);

    Serial.print("rate = ");
    Serial.print((float)IMU.sumCount/IMU.sum, 2);
    Serial.println(" Hz");
    Serial.println("");
  }

  Point next_pos;
  int next_rot = rot;
  GetNextPosRot(&next_pos, &next_rot);
  ReviseScreen(next_pos, next_rot);
  //--
  M5.Lcd.setCursor(0, 100);
  M5.Lcd.printf("   yaw: % 5.2f", IMU.yaw);
  M5.Lcd.setCursor(0, 120);
  M5.Lcd.printf(" pitch: % 5.2f", IMU.pitch);
  M5.Lcd.setCursor(0, 140);
  M5.Lcd.printf("  roll: % 5.2f", IMU.roll);
  //--
  M5.update();
  delay(game_speed);                                      // SPEED ADJUST
}
//========================================================================
void Draw() {                               // Draw 120x240 in the center
  for (int i = 0; i < Width; ++i) for (int j = 0; j < Height; ++j)
   for (int k = 0; k < Length; ++k) for (int l = 0; l < Length; ++l)
    backBuffer[j * Length + l][i * Length + k] = BlockImage[screen[i][j]][k][l];
    M5.Lcd.drawBitmap(100, 0, 120, 240, (uint8_t*)backBuffer);
}
//========================================================================
void PutStartPos() {
  pos.X = 4; pos.Y = 1;
  block = blocks[random(7)];
  rot = random(block.numRotate);
}
//========================================================================
bool GetSquares(Block block, Point pos, int rot, Point* squares) {
  bool overlap = false;
  for (int i = 0; i < 4; ++i) {
    Point p;
    p.X = pos.X + block.square[rot][i].X;
    p.Y = pos.Y + block.square[rot][i].Y;
    overlap |= p.X < 0 || p.X >= Width || p.Y < 0 || p.Y >= 
      Height || screen[p.X][p.Y] != 0;
    squares[i] = p;
  }
  return !overlap;
}
//========================================================================
void GameOver() {
  for (int i = 0; i < Width; ++i) for (int j = 0; j < Height; ++j)
  if (screen[i][j] != 0) screen[i][j] = 4;
  gameover = true;
}
//========================================================================
void ClearKeys() { but_A=false; but_LEFT=false; but_RIGHT=false;}
//========================================================================
bool KeyPadLoop(){
  if(M5.BtnA.wasPressed()){ClearKeys();but_LEFT =true;return true;}
  if(M5.BtnB.wasPressed()){ClearKeys();but_RIGHT=true;return true;}
  if(M5.BtnC.wasPressed()){ClearKeys();but_A    =true;return true;}
  return false;
}
//========================================================================
void GetNextPosRot(Point* pnext_pos, int* pnext_rot) {
  bool received = KeyPadLoop();
  if (but_A) started = true;
  if (!started) return;
  pnext_pos->X = pos.X;
  pnext_pos->Y = pos.Y;
  if ((fall_cnt = (fall_cnt + 1) % 10) == 0) pnext_pos->Y += 1;
  else if (received) {
    if (but_LEFT) { but_LEFT = false; pnext_pos->X -= 1;}
    else if (but_RIGHT) { but_RIGHT = false; pnext_pos->X += 1;}
    else if (but_A) { but_A = false;
      *pnext_rot = (*pnext_rot + block.numRotate - 1)%block.numRotate; 
    }
  }
}
//========================================================================
void DeleteLine() {
  for (int j = 0; j < Height; ++j) {
    bool Delete = true;
    for (int i = 0; i < Width; ++i) if (screen[i][j] == 0) Delete = false;
    if (Delete) for (int k = j; k >= 1; --k) 
    for (int i = 0; i < Width; ++i) screen[i][k] = screen[i][k - 1];
  }
}
//========================================================================
void ReviseScreen(Point next_pos, int next_rot) {
  if (!started) return;
  Point next_squares[4];
  for (int i = 0; i < 4; ++i) screen[pos.X + 
    block.square[rot][i].X][pos.Y + block.square[rot][i].Y] = 0;
  if (GetSquares(block, next_pos, next_rot, next_squares)) {
   for (int i = 0; i < 4; ++i){
     screen[next_squares[i].X][next_squares[i].Y] = block.color;
   }
   pos = next_pos; rot = next_rot;
  }
  else {
   for (int i = 0; i < 4; ++i) screen[pos.X + 
    block.square[rot][i].X][pos.Y + block.square[rot][i].Y] = block.color;
   if (next_pos.Y == pos.Y + 1) {
    DeleteLine(); PutStartPos();
    if (!GetSquares(block, pos, rot, next_squares)) {
     for (int i = 0; i < 4; ++i) screen[pos.X + 
      block.square[rot][i].X][pos.Y + block.square[rot][i].Y] = block.color;
      GameOver();
    }
   }
  }
  Draw();
}
//========================================================================
void make_block( int n , uint16_t color ){            // Make Block color       
  for ( int i =0 ; i < 12; i++ ) for ( int j =0 ; j < 12; j++ ){
    BlockImage[n][i][j] = color;                           // Block color
    if ( i == 0 || j == 0 ) BlockImage[n][i][j] = 0;       // BLACK Line
  } 
}
//========================================================================

float Ticks[10][2]

float potReadings [11][2]{
};

uint32_t pulses[5000][3]{
};

volatile uint32_t CaptureCountB, Duty, V, c, PeriodA_SL, PeriodB, PeriodB_SL, OldPeriodA, OldPeriodB, CaptureCountA, CaptureCountA_;
volatile boolean CaptureFlagA, CaptureFlagB, RevTick, Processing;
bool RotDirection;
volatile int32_t SamplesA_SL, SamplesB_SL, SamplesA, SamplesB, StartTimeA, StartTimeB;
float AvPeriod, OldSamplesA, OldSamplesB, OldSamplesA_SL, OldAngle;
float TimeNow, TimeNowAccel, LastTime, LastTimeAccel, LastTimeAccel1, dt, phase_var, phase, Speed_var, Speed_var1, PeriodA,
      delta_theta, delta_speed, Count, ticks, Accel, OldSpeed, Speed, Ki = 30000, Kp = 1000, Kd = 10000, KiA = 25000, KpA = 1000, KiAQ = 13000, KpAQ = 1000;


float speedProfile[154][2]{
{  -2  , -1.666667 } ,
{ -0.375  , -4.166667 } ,
{ 1.1875  , -3.750001 } ,
{ 0.4375  , -1.875  } ,
{ -0.5625 , -1.458333 } ,
{ -1  , -0.833333 } ,
{ -1  , 0 } ,
{ -1.375  , 0 } ,
{ -1.375  , 0 } ,
{ -1.375  , -0.833334 } ,
{ -1.625  , -3.125001 } ,
{ -1.8125 , -6.458334 } ,
{ -2  , -10.208337  } ,
{ -2.625  , -14.218754  } ,
{ -2.8125 , -18.48959 } ,
{ -3  , -22.708336  } ,
{ -3.25 , -26.09375 } ,
{ -3  , -29.166672  } ,
{ -2.625  , -31.354172  } ,
{ -2.625  , -34.166672  } ,
{ -2.4375 , -37.447918  } ,
{ -2.4375 , -40.312508  } ,
{ -2.1875 , -42.812511  } ,
{ -2  , -45.260429  } ,
{ -1.8125 , -47.552097  } ,
{ -1.625  , -49.114605  } ,
{ -1.625  , -50.885441  } ,
{ -1.625  , -51.718773  } ,
{ -1.375  , -53.177116  } ,
{ -1.1875 , -54.375034  } ,
{ -1  , -54.791698  } ,
{ -0.8125 , -55.26046 } ,
{ -1  , -55.885452  } ,
{ -1  , -56.562527  } ,
{ -1  , -53.229198  } ,
{ -0.8125 , -54.062523  } ,
{ -0.8125 , -55.000015  } ,
{ -1  , -56.093765  } ,
{ -1.375  , -58.229183  } ,
{ -2  , -60.156269  } ,
{ -2  , -62.604187  } ,
{ -2  , -64.791679  } ,
{ -1.8125 , -66.927094  } ,
{ -1.8125 , -63.177094  } ,
{ -1.8125 , -62.656258  } ,
{ -2  , -63.28125 } ,
{ -2.4375 , -63.125008  } ,
{ -2.4375 , -59.375008  } ,
{ -2.4375 , -62.500008  } ,
{ -2.1875 , -66.145844  } ,
{ -2.1875 , -70.104179  } ,
{ -2.1875 , -74.062508  } ,
{ -2.1875 , -76.718773  } ,
{ -2  , -78.020859  } ,
{ -1.8125 , -80.000023  } ,
{ -1.9375 , -82.031273  } ,
{ -1.8125 , -77.708344  } ,
{ -1.8125 , -77.604187  } ,
{ -2.375  , -76.5625  } ,
{ -2.5  , -76.666679  } ,
{ -2.3125 , -76.510414  } ,
{ -2.25 , -77.031235  } ,
{ -2.125  , -72.760414  } ,
{ -2.1875 , -73.750008  } ,
{ -2.125  , -70.15625 } ,
{ -2.125  , -72.239586  } ,
{ -2.375  , -73.802086  } ,
{ -2.375  , -75.572914  } ,
{ -2.375  , -77.968735  } ,
{ -2.625  , -80.729164  } ,
{ -2.875  , -82.968735  } ,
{ -2.375  , -80.364571  } ,
{ -3.0625 , -81.614571  } ,
{ -3  , -79.479164  } ,
{ -2.8125 , -80.572929  } ,
{ -2.375  , -80.781265  } ,
{ -1.4375 , -79.895844  } ,
{ -0.6875 , -79.062515  } ,
{ 1 , -77.552094  } ,
{ 2.625 , -72.552094  } ,
{ 3.25  , -66.614594  } ,
{ 3.25  , -59.687508  } ,
{ 3.25  , -56.458344  } ,
{ 3 , -51.979168  } ,
{ 4.875 , -47.08334 } ,
{ 6.5 , -39.635422  } ,
{ 8.125 , -33.281246  } ,
{ 8.125 , -26.510412  } ,
{ 7.5 , -18.697916  } ,
{ 8.375 , -11.510417  } ,
{ 11.0625 , -1.197917 } ,
{ 12.375  , 9.739582  } ,
{ 12.1875 , 23.072912 } ,
{ 11.75 , 36.770828 } ,
{ 12.5  , 51.979156 } ,
{ 14.1875 , 64.427063 } ,
{ 16.875  , 74.010391 } ,
{ 21.9375 , 79.479141 } ,
{ 27.25 , 79.583305 } ,
{ 32.8125 , 78.385391 } ,
{ 37.5625 , 76.718727 } ,
{ 41.8125 , 70.520821 } ,
{ 45.6875 , 70.729156 } ,
{ 49  , 73.333336 } ,
{ 52.5625 , 76.145821 } ,
{ 55.875  , 79.374985 } ,
{ 58.875  , 81.979156 } ,
{ 61.625  , 85.208321 } ,
{ 64.125  , 87.395805 } ,
{ 66.9375 , 89.791649 } ,
{ 69.375  , 85.312485 } ,
{ 71.6875 , 85.260391 } ,
{ 73.625  , 84.427071 } ,
{ 74.625  , 87.239555 } ,
{ 75.25 , 90.624985 } ,
{ 75.4375 , 92.708321 } ,
{ 75.625  , 95.416634 } ,
{ 75.5  , 97.812469 } ,
{ 74.9375 , 100.885384  } ,
{ 74.0625 , 102.656235  } ,
{ 73.8125 , 104.218719  } ,
{ 73.375  , 103.437469  } ,
{ 72.375  , 104.739548  } ,
{ 71.3125 , 106.406219  } ,
{ 69.75 , 106.197899  } ,
{ 68  , 106.093719  } ,
{ 65.6875 , 105.520813  } ,
{ 62.3125 , 105.729149  } ,
{ 57.625  , 105.729149  } ,
{ 51.5  , 105.937485  } ,
{ 44.8125 , 106.614563  } ,
{ 38.0625 , 102.604149  } ,
{ 31.3125 , 102.812485  } ,
{ 25.1875 , 100.729134  } ,
{ 19.875  , 97.916634 } ,
{ 15  , 93.95829  } ,
{ 10.5625 , 84.583282 } ,
{ 7.0625  , 82.291618 } ,
{ 4.875 , 76.249962 } ,
{ 3.8125  , 76.874962 } ,
{ 3.25  , 70.312477 } ,
{ 2.1875  , 64.27079  } ,
{ 1.375 , 58.020798 } ,
{ -8.875  , 51.458302 } ,
{ -11.0625  , 46.041637 } ,
{ -5.1875 , 33.645802 } ,
{ -18.6875  , 27.031229 } ,
{ -13.125 , 17.031237 } ,
{ -0.5625 , 14.010407 } ,
{ -1  , 10.833324 } ,
{ -0.1875 , 6.041663  } ,
{ 0.9375  , 2.499999  } ,
{ 1.1875  , 0 } ,
{ 0.1875  , 0 } 
};


float speedProfile2[88][2]{
{  -17.5 , -5.625  } ,
{ -15.1875  , -21.510416  } ,
{ -9.5  , -34.635414  } ,
{ -8.625  , -47.135414  } ,
{ -10 , -60.312496  } ,
{ -10.6875  , -74.270828  } ,
{ -9.6875 , -81.770828  } ,
{ -8.8125 , -91.927078  } ,
{ -7.375  , -101.770828 } ,
{ -7.1875 , -111.5625 } ,
{ -6.3125 , -118.958336 } ,
{ -5.4375 , -125.885422 } ,
{ -5.125  , -131.979172 } ,
{ -4.6875 , -130.416672 } ,
{ -3.6875 , -137.34375  } ,
{ -3.4375 , -134.322922 } ,
{ -3.4375 , -137.291672 } ,
{ -2.8125 , -139.166672 } ,
{ -3.125  , -141.145828 } ,
{ -3.125  , -143.020828 } ,
{ -2.625  , -146.458328 } ,
{ -2.375  , -148.854172 } ,
{ -2.125  , -141.822922 } ,
{ -1.1875 , -141.510422 } ,
{ -0.9375 , -136.770828 } ,
{ -1.625  , -134.21875  } ,
{ -1.1875 , -132.395828 } ,
{ -1.1875 , -130.572906 } ,
{ -0.4375 , -129.114578 } ,
{ -1.625  , -129.375  } ,
{ -1.625  , -129.947906 } ,
{ 0.1875  , -131.354172 } ,
{ 1.1875  , -130.46875  } ,
{ 2.1875  , -124.947914 } ,
{ 3.8125  , -115.729164 } ,
{ 4.9375  , -105.15625  } ,
{ 6.3125  , -92.760422  } ,
{ 7.0625  , -79.114586  } ,
{ 8.75  , -65.052086  } ,
{ 12.9375 , -50.833336  } ,
{ 15.125  , -39.479168  } ,
{ 17.4375 , -29.479168  } ,
{ 17.4375 , -19.479168  } ,
{ 16.4375 , -6.927083 } ,
{ 15.25 , 9.270833  } ,
{ 14  , 29.53125  } ,
{ 12.5625 , 50  } ,
{ 13.625  , 69.166664 } ,
{ 15.9375 , 85.520836 } ,
{ 19.375  , 97.34375  } ,
{ 23.8125 , 104.635414  } ,
{ 29.375  , 105.937492  } ,
{ 36.125  , 104.270828  } ,
{ 43.9375 , 102.291664  } ,
{ 50.625  , 102.760414  } ,
{ 56.9375 , 104.374992  } ,
{ 64.0625 , 106.927078  } ,
{ 71  , 109.114578  } ,
{ 76.9375 , 111.09375 } ,
{ 82.25 , 112.96875 } ,
{ 86.6875 , 115.625 } ,
{ 90.4375 , 118.4375  } ,
{ 93.125  , 120.833336  } ,
{ 94.5  , 123.593758  } ,
{ 95  , 126.822922  } ,
{ 94  , 126.666672  } ,
{ 90.8125 , 128.645844  } ,
{ 85.5625 , 130.677078  } ,
{ 79.6875 , 131.822922  } ,
{ 72.5625 , 132.864578  } ,
{ 65.625  , 133.958328  } ,
{ 59  , 134.0625  } ,
{ 52.25 , 135 } ,
{ 45.75 , 136.041656  } ,
{ 39  , 133.697906  } ,
{ 31.6875 , 132.760406  } ,
{ 23.9375 , 129.21875 } ,
{ 16.4375 , 124.843742  } ,
{ 9.9375  , 118.177078  } ,
{ 5.875 , 107.864578  } ,
{ 4.0625  , 97.239578 } ,
{ 3.4375  , 92.135414 } ,
{ 1.375 , 84.166664 } ,
{ 0.1875  , 75.104164 } ,
{ -5.625  , 62.8125 } ,
{ -4.5625 , 49.0625 } ,
{ -14 , 33.28125  } ,
{ -12.25  , 18.645834 } 
};

//Low pass butterworth filter order=1 alpha1=5 

float startTime;
int myIndex;
float prevRadPerS;
unsigned long prevRPMtime;
int i = 0;
int x = 0;
float FWspeed;
float virHandSpeed;
bool Uncouple = false;
float takeTime;
int a = 0;
bool justHit = true;
unsigned long startTime1;
float PotReading;
float t = 0;
float avForce;
float avCurrent;
float oldCurrent;
float SoftStart;
float EncoderRPM;
float EncoderRPMA;
float EncoderRPMB;
float EncoderAccel;
float OldEncoderRPM;
float PredictedRPM;
float SumError;
float Error;
float TimeHere;
float FWengageSpeed = 1;

typedef struct SYSTEM_PARAM_T{
  float motorHandleRatio;
  float motorKt;
  float hullDragConstant;
  float mass;
  float cDamp;
  float cIn;
  float Inertia;
};

typedef struct OAR_T{
  float outboardLength;
  float inboardLength;
  float surfaceConstant;
  float stiffness;
  float mass;
  float inertia;
};

typedef struct BOAT_STATE_T{
  float boatSpeed;
  float boatAccel;
  float handlePosition;
  float handleRadPerS;
  float handleAccel;
  float bladeDepth;
  float spoonRadPerS;
  float spoonForce;
  float spoonAngle;
  float spoonSpeed;
  float deflection;
  float deflectionVel;
  float handleForce;
  float bstimer;
  unsigned long deflTimer;
  float boatEnergy;
  float relSpeed;
  float handleSpeed;
  float FWspeed;
  float FWspeedReal;
  float FWspeedPrev;
  float FWaccel;
  float transducerForce;
};

typedef struct MOTOR_STATE_T{
  float RPM;
  float radPerS;
  float radPerSInertia;
  unsigned long RPMtime;
  float current;
  float accel;
  float accelInertia;
  float revs;
  unsigned long prevRPMtime;
  float InertialOffsetTorque;
  float duty;
  long dutyRPM;
  float dutyAccel;
  float radPerSCorrected;
};

BOAT_STATE_T boat = {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
MOTOR_STATE_T motor = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
OAR_T oar = {2.600, 1.150, 200, 100000, 2, 0}; //was 0.00015
SYSTEM_PARAM_T systemParam = {15, 0.15, 500, 90, 50, 0.076, 0.00087}; //was 2.77 energy bleed

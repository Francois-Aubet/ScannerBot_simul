
#define numberOfScales 10

#define PI 3.1415926535897932384626433832795

#define sizeOfDescript 32

#define precision 0.01
#define maxRange 30.0
//#define numberOfspot (int)(maxRange / precision)

//#define step4 1

#define simplified 2

#define averageNumberOfLaser 320

#ifdef simplified
    #define numberOfspot 2
#else
    #define numberOfspot (int)(maxRange / precision)
#endif

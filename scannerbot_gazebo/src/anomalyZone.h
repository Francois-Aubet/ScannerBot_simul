#include "main.h"

class AnomalyZone
{

public:

    AnomalyZone(double rays[], int begi, int leng);

    int startIndex, endIndex;

    int length;

    double highestDerivate;

    int static const maxNumbOfPose = 2;
    double rayDist[maxNumbOfPose];

    double derivates[maxNumbOfPose - 1];

private:







};

#include "main.h"

class NeuralPosition
{

public:
    //double descriptor[sizeOfDescript][numberOfspot][numberOfScales];


    NeuralPosition();

    void buildDescri(double ranges[]);

    void print(int scale);

    void buildNextScale();

    void buildAllScales();

    double computeSimilarity(NeuralPosition desrciA, int scaleComp);

private:
    int sample(double a);

    double descriptor[sizeOfDescript][numberOfspot][numberOfScales];

    int highestScaleBuild;

    int static const gaussDisbSensLength = 60; //9;
    double static gaussDistribSensor[gaussDisbSensLength];

    int static const gaussDisbPlaCelLength = 60; //21;
    double static gaussDistribPlaCells[gaussDisbPlaCelLength];


};

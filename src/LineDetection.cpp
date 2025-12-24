#include <LineDetection.h>
#include <cstring>
#include <cmath>
#include <trig.h>

LineDetection::LineDetection() : points{
    {86.5, 0}, {85.63, -12.219}, {80.169, -18.904}, {71.206, -24.997},
    {64.174, -35.019}, {57.142, -45.062}, {50.110, -55.104}, {43.602, -65.388},
    {44.506, -74.275}, {33.562, -79.777}, {21.954, -83.689}, {9.912, -85.394},
    {-2.326, -86.469}, {-14.518, -85.282}, {-26.423, -82.397}, {-37.805, -77.871},
    {-47.732, -71.282}, {-45.747, -61.336}, {-52.779, -51.293}, {-59.811, -41.251},
    {-66.843, -31.208}, {-73.874, -21.166}, {-84.136, -19.660}, {-86.166, -7.597},
    {-86.375, 4.650}, {-84.852, 16.804}, {-75.774, 18.931}, {-68.537, 28.788},
    {-61.506, 38.830}, {-54.474, 48.873}, {-47.442, 58.915}, {-45.728, 69.253},
    {-40.440, 76.541}, {-31.672, 72.424}, {-21.946, 65.169}, {-9.933, 63.291},
    {2.326, 63.291}, {14.583, 63.379}, {26.052, 67.343}, {34.246, 76.291},
    {44.465, 74.207}, {43.602, 65.388}, {50.110, 55.104}, {57.142, 45.062},
    {64.174, 35.019}, {71.206, 24.977}, {80.169, 18.904}, {85.633, 12.219}
} {
    adc1.begin(cs1,mosi,miso,sck);
    adc2.begin(cs2,mosi,miso,sck);
    adc3.begin(cs3,mosi,miso,sck);
    adc4.begin(cs4,mosi,miso,sck);
    adc5.begin(cs5,mosi,miso,sck);
    adc6.begin(cs6,mosi,miso,sck);
    
    adcList[0] = &adc1;
    adcList[1] = &adc2;
    adcList[2] = &adc3;
    adcList[3] = &adc4;
    adcList[4] = &adc5;
    adcList[5] = &adc6;
}

void LineDetection::lineSensorDebug() {
    int chBig = 12;
    for (int i = 0; i < 48; i++) {
        if (i <= 11) {
            chBig = (12-i);
        } else {
            chBig = (48 - (i-12));
        }
        // Serial.print("ADC NUM: ");
        // Serial.println((chBig - 1) % 8);
        // Serial.print("Sensor Read: ");
        // Serial.println((int)((chBig - 1)/8) + 1);

        Serial.println("Sensor " + String(i) + ": " + String(adcList[(chBig - 1) / 8]->analogRead((int)((chBig - 1)/8) + 1)));
    }

}

void LineDetection::updateLineSensors() {
    int chBig = 36;
    for (int i = 0; i < 48; i++) {
        if (i <= 12) {
            chBig = (36+i);
        } else {
            chBig = i-12;
        }

        // FOR DEBUGGING PURPOSES: 
        sensorVals[i] = adcList[(chBig - 1) / 8]->analogRead((int)((chBig - 1)/8) + 1);

       if (sensorVals[i] > calibrateVals[i]+40) {
            activatedVals[i] = 1;
        } else {
            activatedVals[i] = 0;
       }
    }
}



double LineDetection::getLineAngle() {
    updateLineSensors();
    // std::vector<int> pos;
    // for (int i = 0; i < 48; i++) {
    //     if (activatedVals[i] == 1)
    //         pos.push_back(i);
            
    // }

    // int sensNum1 = -1, sensNum2 = -1;
    // int bestDist = -1;
    
    // for (int i = 0; i < (int)pos.size(); i++) {
    //     for (int j = i + 1; j < (int)pos.size(); j++) {
    //         int dist = Trig::getDist(points[pos[i]], points[pos[j]]);
    //         if (dist > bestDist) {
    //             bestDist = dist;
    //             sensNum1 = pos[i];
    //             sensNum2 = pos[j];
    //         }
    //     }
    // }
    // Serial.print("Sens 1: ");
    // Serial.println(sensNum1);
    // Serial.print("Sens 2: ");
    // Serial.println(sensNum2);
    // Point p1 = points[sensNum1];
    // Point p2 = points[sensNum2];
    // Serial.print("Point 1: (");
    // Serial.print(p1.x);
    // Serial.print(", ");
    // Serial.print(p1.y);
    // Serial.println(")");

    // Serial.print("Point 2: (");
    // Serial.print(p2.x);
    // Serial.print(", ");
    // Serial.print(p2.y);
    // Serial.println(")");
    double xTotal = 0;
    double yTotal = 0;
    int count = 0;
    for(int i =0; i<48; i++) {
        if(activatedVals[i]==1) {
            xTotal += points[i].x;
            yTotal += points[i].y;
            count++;
        }
    }
    if(count > 0) {
        xTotal /= count;
        yTotal /= count;
    }
    // Doing Inverse Recipricol in the Function
    double angle = atan2(xTotal*-1, yTotal);
    // double perpendicularSlope = -1 / slope;

    // double dotProd = perpendicularSlope;
    // double mags = sqrt(1 + pow(perpendicularSlope,2));
    // Convert to Radians
    angle *= 180/M_PI;
    angle = (angle<0) ? angle+360 : angle;
    return angle;

    // double angle = fmod((90 - atan2(perpendicularSlope, 1) * (180/M_PI)), 360);

    // return angle;
}
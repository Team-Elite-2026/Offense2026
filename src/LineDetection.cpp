#include <LineDetection.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <trig.h>

Point LineDetection::points[48] = {
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
};

double LineDetection::magnitudes[48] = {};

LineDetection::LineDetection() : angle(-5), cordLength(-5), crossLine(false), prevAngle(-5) {
    for (int i = 0; i < 48; i++) {
        magnitudes[i] = Trig::getDist(points[i], {0,0});
    }
    maxSensorPairDistance = 0.0;
    for (int i = 0; i < 48; ++i) {
        for (int j = i + 1; j < 48; ++j) {
            double d = Trig::getDist(points[i], points[j]);
            if (d > maxSensorPairDistance) {
                maxSensorPairDistance = d;
            }
        }
    }
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



// Helper function to calculate eigenvalues and eigenvectors for 2x2 covariance matrix
// Returns the eigenvector corresponding to the largest eigenvalue
Point calculatePrincipalComponent(double cov_xx, double cov_xy, double cov_yy) {
    // For 2x2 matrix [a b; c d] = [cov_xx cov_xy; cov_xy cov_yy]
    // Eigenvalues: lambda = (trace ± sqrt(trace^2 - 4*det)) / 2
    double trace = cov_xx + cov_yy;
    double det = cov_xx * cov_yy - cov_xy * cov_xy;
    double discriminant = trace * trace - 4 * det;
    
    if (discriminant < 0) {
        // Shouldn't happen for covariance matrix, but handle it
        return {1.0, 0.0};
    }
    
    double lambda1 = (trace + sqrt(discriminant)) / 2.0;
    double lambda2 = (trace - sqrt(discriminant)) / 2.0;
    
    // Use the eigenvalue with larger magnitude
    double lambda = (fabs(lambda1) > fabs(lambda2)) ? lambda1 : lambda2;
    
    // Calculate eigenvector for this eigenvalue
    // (A - lambda*I) * v = 0
    // For 2x2: [a-lambda b; c d-lambda] * [vx; vy] = 0
    // We can use: if b != 0, then vx = 1, vy = -(a-lambda)/b
    // Or if (d-lambda) != 0, then vy = 1, vx = -b/(d-lambda)
    Point eigenvec;
    double a_minus_lambda = cov_xx - lambda;
    
    if (fabs(cov_xy) > 1e-6) {
        eigenvec.x = 1.0;
        eigenvec.y = -a_minus_lambda / cov_xy;
    } else if (fabs(cov_yy - lambda) > 1e-6) {
        eigenvec.y = 1.0;
        eigenvec.x = -cov_xy / (cov_yy - lambda);
    } else {
        // Degenerate case
        eigenvec.x = 1.0;
        eigenvec.y = 0.0;
    }
    
    // Normalize
    double mag = sqrt(eigenvec.x * eigenvec.x + eigenvec.y * eigenvec.y);
    if (mag > 1e-6) {
        eigenvec.x /= mag;
        eigenvec.y /= mag;
    }
    
    return eigenvec;
}

void LineDetection::updateLineSensors(bool withDebug) {
    int chBig = 36;
    for (int i = 0; i < 48; i++) {
        if (i <= 12) {
            chBig = (36+i);
        } else {
            chBig = i-12;
        }

        
        sensorVals[i] = adcList[(chBig - 1) / 8]->analogRead((int)((chBig - 1)%8));
        
        // Removed individual sensor printing - too slow
        // Angle is sent separately from Calculate() function

       if (sensorVals[i] > calibrateVals[i]+30) {
            activatedVals[i] = 1;
        } else {
            activatedVals[i] = 0;
       }
    }
}



void LineDetection::Calculate() {
    updateLineSensors(false); // Make true if you want to print out all line sensor values for GUI Debug
    
    std::vector<int> pos;
    double xTotal = 0;
    double yTotal = 0;
    int count = 0;
    for(int i =0; i<48; i++) {
        if(activatedVals[i]==1) {
            pos.push_back(i);
            xTotal += points[i].x;
            yTotal += points[i].y;
            count++;
        }
    }

    if (count > 0) {

        xTotal /= count;
        yTotal /= count;

        // Calculate Centroid Distance to Origin 
        Point centroid;
        Point origin;
        centroid.x = xTotal;
        centroid.y = yTotal;
        origin.x = 0;
        origin.y = 0;

        cordLength = 1 - Trig::getDist(centroid, origin)/89.0; // Divide by the radius of the ideal circle
        Serial.println("Centroid: (" + String(centroid.x) + ", " + String(centroid.y) + ")");
        Serial.println("Cord Length: " + String(cordLength));
        
        // Orthogonal regression (PCA) if we have at least 2 points
        if (count >= 2) {
            // Use already calculated centroid (xTotal, yTotal)
            double mean_x = xTotal;
            double mean_y = yTotal;
            
            // Calculate covariance matrix
            double cov_xx = 0, cov_xy = 0, cov_yy = 0;
            for (int idx : pos) {
                double dx = points[idx].x - mean_x;
                double dy = points[idx].y - mean_y;
                cov_xx += dx * dx;
                cov_xy += dx * dy;
                cov_yy += dy * dy;
            }
            cov_xx /= count;
            cov_xy /= count;
            cov_yy /= count;
            
            // Get principal component (direction of line)
            Point direction = calculatePrincipalComponent(cov_xx, cov_xy, cov_yy);
            
            // Convert to slope-intercept form: y = mx + b
            if (fabs(direction.x) > 1e-6) {  // Non-vertical line
                double slope = direction.y / direction.x;
                double intercept = mean_y - slope * mean_x;
                
                // Calculate perpendicular line from origin (robot position)
                double m_perp = -1.0 / slope;
                
                // Find intersection: mx + b = m_perp * x
                // x(m - m_perp) = -b
                double slope_diff = slope - m_perp;
                if (fabs(slope_diff) > 1e-6) {
                    double x_intersect = -intercept / slope_diff;
                    double y_intersect = m_perp * x_intersect;
                    
                    // Calculate angle from front (0° = +Y direction)
                    // atan2(x, y) gives angle where 0° is +Y, 90° is +X
                    angle = atan2(x_intersect, y_intersect) * 180.0 / M_PI;
                    
                    // Normalize to 0-360 range
                    if (angle < 0) {
                        angle += 360;
                    }
                    if (angle >= 360) {
                        angle -= 360;
                    }
                } else {
                    // Lines are parallel - fallback to centroid angle
                    angle = atan2(mean_x, mean_y) * 180.0 / M_PI;
                    if (angle < 0) angle += 360;
                    if (angle >= 360) angle -= 360;
                }
            } else {
                // Vertical line case - intersection is directly left/right
                double x_intersect = mean_x;
                double y_intersect = 0;  // Robot is at origin
                angle = (x_intersect > 0) ? 90.0 : 270.0;
            }
        } else {
            // Only 1 point detected - use centroid angle
            angle = atan2(xTotal, yTotal) * 180.0 / M_PI;
            if (angle < 0) angle += 360;
            if (angle >= 360) angle -= 360;
        }
    } else {
        crossLine = false;
        angle = -5;
        prevAngle = angle;
        cordLength = -5;
    }
}
double LineDetection::getAngle() {
    return angle;
}

double LineDetection::avoidanceAngle() {
    double diff = abs(prevAngle - angle);
    double circularDist = std::min(diff, 360.0 - diff);
    if (circularDist > 150 && prevAngle != -5) {
        crossLine = !crossLine;
    }
    
    prevAngle = angle;

    if (crossLine) {
        return angle;
    }

    double newAngle = angle + 180;
    return (newAngle > 360) ? (newAngle - 360) : (newAngle);
}

bool LineDetection::getCrossLine() const {
    return crossLine;
}

double LineDetection::desiredPerpendicularHeadingFromLine(double lineAngleDeg) {
    if (lineAngleDeg == -5) {
        return 0.0;
    }

    double relNormalA = Trig::wrapAngle(lineAngleDeg);
    double relNormalB = Trig::wrapAngle(lineAngleDeg + 180.0);
    return (fabs(relNormalA) <= fabs(relNormalB)) ? relNormalA : relNormalB;
}

double LineDetection::getCordLength() {
    return cordLength;
}

double LineDetection::getChordLengthFurthestPairNormalized() {
    int idx[48];
    int k = 0;
    for (int i = 0; i < 48; ++i) {
        if (activatedVals[i] == 1) {
            idx[k++] = i;
        }
    }
    if (k < 2) {
        return -5.0;
    }

    double maxD2 = 0.0;
    for (int i = 0; i < k; ++i) {
        const Point& pi = points[idx[i]];
        for (int j = i + 1; j < k; ++j) {
            const Point& pj = points[idx[j]];
            double dx = pi.x - pj.x;
            double dy = pi.y - pj.y;
            double d2 = dx * dx + dy * dy;
            if (d2 > maxD2) {
                maxD2 = d2;
            }
        }
    }

    double furthest = sqrt(maxD2);
    double n = furthest / maxSensorPairDistance;
    if (n > 1.0) {
        n = 1.0;
    }
    return n;
}

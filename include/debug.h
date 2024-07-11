#ifndef DEBUG_H
#define DEBUG_H


#include <vector>
#include <numeric> // Para std::accumulate

using namespace std;

extern vector<double> speedD;
extern vector<double> speedE;


float calculateAverage(const vector<double>& speeds) {
    if (speeds.empty()) {
        return 0.0f; // Evita divisão por zero
    }
    float sum = accumulate(speeds.begin(), speeds.end(), 0.0f);
    return sum / speeds.size();
}

#endif
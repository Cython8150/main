#include "iostream"
#include "RemoteAPIClient.h"

using namespace std;

int main() {
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    sim.setStepping(true);

    sim.startSimulation();
    double t = 0.0;

    do{
        t = sim.getSimulationTime();
        printf("Simulation time: %.2f [s]\n", t);
        sim.step();
    } while(t < 10.0);
    sim.stopSimulation();
    return(0);
}
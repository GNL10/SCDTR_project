#include "consensus.h"

// case 1
int L1 = 150, o1 = 30, L2 = 80, o2 = 0;
// case 2
//int L1 = 80, o1 = 50, L2 = 150, o2 = 50;
// case 3
//int L1 = 80, o1 = 50, L2 = 270, o2 = 50;

// symmetric costs
int cost_1 = 1, cost_2 = 1;
// asymmetric costs
//int cost_1 = 1, cost_2 = 3;

float k11 = 2, k12 = 1;
float gains[] {k11, k12};

Consensus* node1;

void setup(){
Serial.begin(115200);
}

void loop(){

node1 = new Consensus(0, L1, o1, gains, cost_1);

float d[] {1, 1} ;
int len = 2; 
float* cost;
node1->evaluate_cost(d, len, cost);
Serial.print("cost: ");
Serial.println(*cost);
delay(5000);

}
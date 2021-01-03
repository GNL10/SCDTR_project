class Node
{
private:
public:
  float d[2] = {0, 0};
  float d_av[2] = {0, 0};
  float y[2] = {0, 0};
  float k[2] = {0, 0};
  float n = 0;
  float m = 0;
  float c[2] = {0, 0};
  float o = 0;
  float L = 0;

  Node(int idx, float gains[], float ext, int cost, int des_lux);
  float sum_squares(float vec[]);
  
};
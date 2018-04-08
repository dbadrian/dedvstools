#include <vector>
#include <iostream>
#include <stdio.h>
#include <cmath>

#include <DEdvs/dedvs_auxiliary.h>
#include <DEdvs/dedvs.h>

#include <Eigen/Dense>

int main(int argc, char *argv[])
{
  std::vector<dedvs::DEvent> data;
  dedvs::DEvent temp;
  double numerator    = 0;
  double denominator  = 0;
  double dist_d, dist_e;
  float pu,pv;
  double err = 0;

  Eigen::Vector3f tp;

  std::cout << "Entries: " << dedvs::readDEventsFromFile(argv[1],&data) << std::endl;

  //std::cout << data[0].x << " " << data[0].y << " " << data[0].u << " " << data[0].v << " " << std::endl; 

  denominator = data.size();

  for(auto tmp : data)
  {
    tp = dedvs::unprojectEDVS(tmp);
    dedvs::projectKinect(tp,&(pu),&(pv));
    //std::cout << tmp.x << " " << tmp.y << " " << pu << " " << pv << " " << std::endl; 
    err += pow(tmp.x-pu,2) + pow(tmp.y-pv,2);
  }

   err = sqrt(err / denominator);

   std::cout << "Error: " << err << std::endl;

  return 0;
}

//18.75
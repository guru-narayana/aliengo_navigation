#ifndef __ALIENKINDYM_H__
#define __ALIENKINDYM_H__

#include <vector>
#include <cmath>

using namespace std;


class quad_kinem
{
public:
    quad_kinem(double L1,double L2,double L3,double Length,double width);
    vector<double> Left_Leg_IK(vector<double> xyz_state);
    vector<double> Right_Leg_IK(vector<double> xyz_state);
    vector<double> Left_Leg_FK(vector<double> cjs);
    vector<double> Right_Leg_FK(vector<double> cjs);
};

class quad_dym{

};

#endif
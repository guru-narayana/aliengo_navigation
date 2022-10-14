#ifndef __SHIFTINGCONTROL_H__
#define __SHIFTINGCONTROL_H__


void height_adjust(ros::Publisher jnt_st_pub);
void shift_mode(ros::Publisher jnt_st_pub);
vector<vector<double>> generate_swing_coefs(vector<double> p_init,vector<double> p_final);
vector<vector<double>> Multiply(vector <vector<double>> &a, vector <vector<double>> &b);
#endif

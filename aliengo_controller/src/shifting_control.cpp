#include "aliengo_controller/controller.h"
#include "aliengo_controller/shifting_control.h"

double getDeterminant(const std::vector<std::vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 
    int dimension = vect.size();

    if(dimension == 0) {
        return 1;
    }

    if(dimension == 1) {
        return vect[0][0];
    }

    //Formula for 2x2-matrix
    if(dimension == 2) {
        return vect[0][0] * vect[1][1] - vect[0][1] * vect[1][0];
    }

    double result = 0;
    int sign = 1;
    for(int i = 0; i < dimension; i++) {

        //Submatrix
        std::vector<std::vector<double>> subVect(dimension - 1, std::vector<double> (dimension - 1));
        for(int m = 1; m < dimension; m++) {
            int z = 0;
            for(int n = 0; n < dimension; n++) {
                if(n != i) {
                    subVect[m-1][z] = vect[m][n];
                    z++;
                }
            }
        }

        //recursive call
        result = result + sign * vect[0][i] * getDeterminant(subVect);
        sign = -sign;
    }

    return result;
}

std::vector<std::vector<double>> getTranspose(const std::vector<std::vector<double>> matrix1) {

    //Transpose-matrix: height = width(matrix), width = height(matrix)
    std::vector<std::vector<double>> solution(matrix1[0].size(), std::vector<double> (matrix1.size()));

    //Filling solution-matrix
    for(size_t i = 0; i < matrix1.size(); i++) {
        for(size_t j = 0; j < matrix1[0].size(); j++) {
            solution[j][i] = matrix1[i][j];
        }
    }
    return solution;
}

std::vector<std::vector<double>> getCofactor(const std::vector<std::vector<double>> vect) {
    if(vect.size() != vect[0].size()) {
        throw std::runtime_error("Matrix is not quadratic");
    } 

    std::vector<std::vector<double>> solution(vect.size(), std::vector<double> (vect.size()));
    std::vector<std::vector<double>> subVect(vect.size() - 1, std::vector<double> (vect.size() - 1));

    for(std::size_t i = 0; i < vect.size(); i++) {
        for(std::size_t j = 0; j < vect[0].size(); j++) {

            int p = 0;
            for(size_t x = 0; x < vect.size(); x++) {
                if(x == i) {
                    continue;
                }
                int q = 0;

                for(size_t y = 0; y < vect.size(); y++) {
                    if(y == j) {
                        continue;
                    }

                    subVect[p][q] = vect[x][y];
                    q++;
                }
                p++;
            }
            solution[i][j] = pow(-1, i + j) * getDeterminant(subVect);
        }
    }
    return solution;
}

vector<vector<double>> getInverse(const vector<vector<double>> vect) {
    if(getDeterminant(vect) == 0) {
        throw std::runtime_error("Determinant is 0");
    } 
    double d = 1.0/getDeterminant(vect);
    vector<vector<double>> solution(vect.size(), std::vector<double> (vect.size()));
    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] = vect[i][j]; 
        }
    }
    solution = getTranspose(getCofactor(solution));
    for(size_t i = 0; i < vect.size(); i++) {
        for(size_t j = 0; j < vect.size(); j++) {
            solution[i][j] *= d;
        }
    }
    return solution;
}
vector<vector<double>> Multiply(vector <vector<double>> &a, vector <vector<double>> &b){
    const int n = a.size();     // a rows
    const int m = a[0].size();  // a cols
    const int p = b[0].size();  // b cols
    vector <vector<double>> c(n, vector<double>(p, 0));
    for (auto j = 0; j < p; ++j)
    {
        for (auto k = 0; k < m; ++k)
        {
            for (auto i = 0; i < n; ++i)
            {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return c;
}
vector<vector<double>> generate_swing_coefs(vector<double> p_init,vector<double> p_final){
    vector<double> p_middle = {(p_init[0]+p_final[0])/2,(p_init[1]+p_final[1])/2,min(p_init[2],p_final[2])+robot_swing_height};
    vector<vector<double>> P_mat = {p_init,p_middle,p_final};
    double z_ratio = p_final[2]/(p_final[2]+p_init[2]);
    vector<vector<double>> B = {{1,0,0},{1,z_ratio,pow(z_ratio,2)},{1,1,1}};
    vector<vector<double>> B_inv = getInverse(B);
    return Multiply(B_inv,P_mat);
}

void height_adjust(ros::Publisher jnt_st_pub){
    double delta_h = robot_base_height-base_pose_z,
    Time = abs(delta_h)/robot_verti_vel,
    init_time = ros::Time::now().toSec();
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    ros::Rate rate(100);
    vector<double>  FL = {current_robot_footsteps[0][0],current_robot_footsteps[0][1],current_robot_footsteps[0][2]},
                FR = {current_robot_footsteps[1][0],current_robot_footsteps[1][1],current_robot_footsteps[0][2]},
                RL = {current_robot_footsteps[2][0],current_robot_footsteps[2][1],current_robot_footsteps[0][2]},
                RR = {current_robot_footsteps[3][0],current_robot_footsteps[3][1],current_robot_footsteps[0][2]};
    while(ros::Time::now().toSec()-init_time<=Time){
        double h = (ros::Time::now().toSec()-init_time)*delta_h/Time + base_pose_z;
        FL[2] = h;FR[2] = h;RL[2] = h;RR[2] = h;
        vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL)),
                        RL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToRL(RL)),
                        FR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToFR(FR)),
                        RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR));
        jnt_set_st.joint_positions[0] = -FR_req_jnt[0];
        jnt_set_st.joint_positions[1] = -FR_req_jnt[1];
        jnt_set_st.joint_positions[2] = -FR_req_jnt[2];
        jnt_set_st.joint_positions[9] = RL_req_jnt[0];
        jnt_set_st.joint_positions[10]= RL_req_jnt[1];
        jnt_set_st.joint_positions[11]= RL_req_jnt[2];
        jnt_set_st.joint_positions[3] = FL_req_jnt[0];
        jnt_set_st.joint_positions[4] = FL_req_jnt[1];
        jnt_set_st.joint_positions[5] = FL_req_jnt[2];
        jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
        jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
        jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
        jnt_st_pub.publish(jnt_set_st);
        rate.sleep();
}
}

void shift_mode(ros::Publisher jnt_st_pub){
    vector<double> endpnt_FL = {robot_config[0]/2,((robot_config[1]/2) + robot_config[2]),-robot_base_height},
                    endpnt_RL = {-robot_config[0]/2,((robot_config[1]/2) + robot_config[2]),-robot_base_height},
                    endpnt_FR = {robot_config[0]/2,-((robot_config[1]/2) + robot_config[2]),-robot_base_height},
                    endpnt_RR = {-robot_config[0]/2,-((robot_config[1]/2) + robot_config[2]),-robot_base_height};
    double T = 0.3;
    quad_kinem quad_kinem_g(robot_config[2],robot_config[3],robot_config[4],robot_config[0],robot_config[1]);
    vector<vector<double>> A_FL = generate_swing_coefs(current_robot_footsteps[0],endpnt_FL), 
                            A_FR = generate_swing_coefs(current_robot_footsteps[1],endpnt_FR), 
                            A_RL = generate_swing_coefs(current_robot_footsteps[2],endpnt_RL), 
                            A_RR = generate_swing_coefs(current_robot_footsteps[3],endpnt_RR);
    double init_time = ros::Time::now().toSec();
    ros::Rate frequency(100);
    while(ros::Time::now().toSec()-init_time<=T){
        double u = (ros::Time::now().toSec()-init_time)/T;
        vector<vector<double>> u_mat = {{1,u,pow(u,2)}};
        vector<vector<double>> FL_cnt_pos = Multiply(u_mat,A_FL),
                                RR_cnt_pos = Multiply(u_mat,A_RR);
        vector<double> FL_req_jnt = quad_kinem_g.Left_Leg_IK(quad_kinem_g.BaseToFL(FL_cnt_pos[0])),
                        RR_req_jnt = quad_kinem_g.Right_Leg_IK(quad_kinem_g.BaseToRR(RR_cnt_pos[0]));
        jnt_set_st.joint_positions[3] = FL_req_jnt[0];
        jnt_set_st.joint_positions[4] = FL_req_jnt[1];
        jnt_set_st.joint_positions[5] = FL_req_jnt[2];
        jnt_set_st.joint_positions[6] = -RR_req_jnt[0];
        jnt_set_st.joint_positions[7] = -RR_req_jnt[1];
        jnt_set_st.joint_positions[8] = -RR_req_jnt[2];
        jnt_st_pub.publish(jnt_set_st);
        frequency.sleep();
        
    }
}
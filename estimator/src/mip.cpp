#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

struct Point3D
{
     float x;
     float y;
     float z;
};

int main(int argc, char **argv)
{
     ros::init(argc, argv, "point_cloud_publisher");
     ros::NodeHandle nh;
     ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);


     std::ifstream file("/home/yuanzhi/catkin_ws/src/estimator/data/ideal_trace.txt");
     std::vector<Point3D> points;
     if (file.is_open())
     {
          std::string line;
          while (std::getline(file, line))
          {
               std::stringstream ss(line);
               std::string token;
               Point3D point;
               // 使用制表符分隔每行的坐标xyz分量
               std::getline(ss, token, '\t');
               point.x = std::stof(token);
               std::getline(ss, token, '\t');
               point.y = std::stof(token);
               std::getline(ss, token, '\t');
               point.z = std::stof(token);
               points.push_back(point);
          }
          file.close();
     }

     ros::Rate rate(30); // 设置发布频率为30Hz
     geometry_msgs::PointStamped pose_msg;
     pose_msg.header.frame_id = "base_link"; // 设置坐标系为base_link

     for (size_t i = 0; i < points.size(); ++i)
     {
          pose_msg.point.x = points[i].x;
          pose_msg.point.y = points[i].y;
          pose_msg.point.z = points[i].z;

          while (ros::ok())
          {
               if (pose_pub.getNumSubscribers() > 0)
               {
                    pose_pub.publish(pose_msg);
                    break;
               }
               ros::spinOnce();
               rate.sleep();
          }
     }

     return 0;
}

// #include <cassert>
// #include "gurobi_c++.h"
// using namespace std;

// int main(int argc,
//          char *argv[])
// {
//     try
//     {
//         GRBEnv env = GRBEnv();

//         GRBModel model = GRBModel(env);

//         // Create var// #include <cassert>
// #include "gurobi_c++.h"
// using namespace std;

// int main(int argc,
//          char *argv[])
// {
//     try
//     {
//         GRBEnv env = GRBEnv();

//         GRBModel model = GRBModel(env);

//         // Create variables

//         GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
//         GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
//         GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");

//         // Set objective

//         GRBLinExpr obj = x;
//         model.setObjective(obj, GRB_MAXIMIZE);

//         // Add linear constraint: x + y + z <= 10

//         model.addConstr(x + y + z <= 10, "c0");

//         // Add bilinear inequality constraint: x * y <= 2

//         model.addQConstr(x * y <= 2, "bilinear0");

//         // Add bilinear equality constraint: y * z == 1

//         model.addQConstr(x * z + y * z == 1, "bilinear1");

//         // First optimize() call will fail - need to set NonConvex to 2

//         model.set(GRB_IntParam_NonConvex, 2);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;
//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//         // Constrain x to be integral and solve again
//         x.set(GRB_CharAttr_VType, GRB_INTEGER);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;

//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//     }
//     catch (GRBException e)
//     {
//         cout << "Error code = " << e.getErrorCode() << endl;
//         cout << e.getMessage() << endl;
//     }
//     catch (...)
//     {
//         cout << "Exception during optimization" << endl;
//     }

//     return 0;
// }

// #include <iostream>
// #include <cmath>
// #include <cstdlib>
// #include <ctime>
// #include "gurobi_c++.h"
// #include <vector>

// #include <ctime>
// #include <unistd.h>
// #include <sstream>

// using namespace std;

// // 定义优化目标函数
// void vio_optimization(const std::vector<float> &imu_acc, const std::vector<float> &s, GRBModel &model, float *position, float *velocity)
// {
//     int n = imu_acc.size(); // IMU数据点数
//     float dt = 1;           // 时间步长
//     GRBVar x[n];            // 机器人位姿
//     GRBVar v[n];            // 机器人位姿
//     // std::vector<GRBVar> v;

//     // 定义优化变量属性：第三个参数表示该变量的系数（0表示不参与目标函数的计算），GRB_CONTINUOUS表示该变量为连续型变量
//     for (int i = 0; i < n; i++)
//     {
//         x[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//         v[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//     }
//     std::cout << "0000000000" << std::endl;
//     // 添加约束条件
//     for (int i = 0; i < n - 1; i++)
//     {
//         // 约束机器人位姿的变化模型
//         model.addConstr(x[i + 1] == x[i] + v[i] * dt + 0.5 * dt * dt * imu_acc[i]);
//         model.addConstr(v[i + 1] == v[i] + dt * imu_acc[i]);
//     }
//     std::cout << "1111111111" << std::endl;

//     v[2].set(GRB_DoubleAttr_X, 0.5);

//     GRBQuadExpr obj; // 定义目标函数（最小化机器人位姿和特征点位置的位置估计误差）

//     for (int i = 0; i < n - 1; i++)
//         obj += (x[i] - s[i]) * (x[i] - s[i]);
//     try
//     {
//         model.setObjective(obj);
//         model.optimize();
//         for (int i = 0; i < n; i++)
//             position[i] = x[i].get(GRB_DoubleAttr_X);
//         for (int j = 0; j < n; j++)
//             velocity[j] = v[j].get(GRB_DoubleAttr_X);
//     }
//     catch (GRBException &ex)
//     {
//         cout << "Error code =" << ex.getErrorCode() << endl;
//         cout << ex.getMessage() << endl;
//     }
// }
iables

//         GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
//         GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
//         GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");

//         // Set objective

//         GRBLinExpr obj = x;
//         model.setObjective(obj, GRB_MAXIMIZE);

//         // Add linear constraint: x + y + z <= 10

//         model.addConstr(x + y + z <= 10, "c0");

//         // Add bilinear inequality constraint: x * y <= 2

//         model.addQConstr(x * y <= 2, "bilinear0");

//         // Add bilinear equality constraint: y * z == 1

//         model.addQConstr(x * z + y * z == 1, "bilinear1");

//         // First optimize() call will fail - need to set NonConvex to 2

//         model.set(GRB_IntParam_NonConvex, 2);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;
//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//         // Constrain x to be integral and solve again
//         x.set(GRB_CharAttr_VType, GRB_INTEGER);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;

//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//     }// #include <cassert>
// #include "gurobi_c++.h"
// using namespace std;

// int main(int argc,
//          char *argv[])
// {
//     try
//     {
//         GRBEnv env = GRBEnv();

//         GRBModel model = GRBModel(env);

//         // Create variables

//         GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
//         GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
//         GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");

//         // Set objective

//         GRBLinExpr obj = x;
//         model.setObjective(obj, GRB_MAXIMIZE);

//         // Add linear constraint: x + y + z <= 10

//         model.addConstr(x + y + z <= 10, "c0");

//         // Add bilinear inequality constraint: x * y <= 2

//         model.addQConstr(x * y <= 2, "bilinear0");

//         // Add bilinear equality constraint: y * z == 1

//         model.addQConstr(x * z + y * z == 1, "bilinear1");

//         // First optimize() call will fail - need to set NonConvex to 2

//         model.set(GRB_IntParam_NonConvex, 2);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;
//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//         // Constrain x to be integral and solve again
//         x.set(GRB_CharAttr_VType, GRB_INTEGER);
//         model.optimize();

//         cout << x.get(GRB_StringAttr_VarName) << " "
//              << x.get(GRB_DoubleAttr_X) << endl;
//         cout << y.get(GRB_StringAttr_VarName) << " "
//              << y.get(GRB_DoubleAttr_X) << endl;
//         cout << z.get(GRB_StringAttr_VarName) << " "
//              << z.get(GRB_DoubleAttr_X) << endl;

//         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//     }
//     catch (GRBException e)
//     {
//         cout << "Error code = " << e.getErrorCode() << endl;
//         cout << e.getMessage() << endl;
//     }
//     catch (...)
//     {
//         cout << "Exception during optimization" << endl;
//     }

//     return 0;
// }

// #include <iostream>
// #include <cmath>
// #include <cstdlib>
// #include <ctime>
// #include "gurobi_c++.h"
// #include <vector>

// #include <ctime>
// #include <unistd.h>
// #include <sstream>

// using namespace std;

// // 定义优化目标函数
// void vio_optimization(const std::vector<float> &imu_acc, const std::vector<float> &s, GRBModel &model, float *position, float *velocity)
// {
//     int n = imu_acc.size(); // IMU数据点数
//     float dt = 1;           // 时间步长
//     GRBVar x[n];            // 机器人位姿
//     GRBVar v[n];            // 机器人位姿
//     // std::vector<GRBVar> v;

//     // 定义优化变量属性：第三个参数表示该变量的系数（0表示不参与目标函数的计算），GRB_CONTINUOUS表示该变量为连续型变量
//     for (int i = 0; i < n; i++)
//     {
//         x[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//         v[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//     }
//     std::cout << "0000000000" << std::endl;
//     // 添加约束条件
//     for (int i = 0; i < n - 1; i++)
//     {
//         // 约束机器人位姿的变化模型
//         model.addConstr(x[i + 1] == x[i] + v[i] * dt + 0.5 * dt * dt * imu_acc[i]);
//         model.addConstr(v[i + 1] == v[i] + dt * imu_acc[i]);
//     }
//     std::cout << "1111111111" << std::endl;

//     v[2].set(GRB_DoubleAttr_X, 0.5);

//     GRBQuadExpr obj; // 定义目标函数（最小化机器人位姿和特征点位置的位置估计误差）

//     for (int i = 0; i < n - 1; i++)
//         obj += (x[i] - s[i]) * (x[i] - s[i]);
//     try
//     {
//         model.setObjective(obj);
//         model.optimize();
//         for (int i = 0; i < n; i++)
//             position[i] = x[i].get(GRB_DoubleAttr_X);
//         for (int j = 0; j < n; j++)
//             velocity[j] = v[j].get(GRB_DoubleAttr_X);
//     }
//     catch (GRBException &ex)
//     {
//         cout << "Error code =" << ex.getErrorCode() << endl;
//         cout << ex.getMessage() << endl;
//     }
// }

//     catch (GRBException e)
//     {
//         cout << "Error code = " << e.getErrorCode() << endl;
//         cout << e.getMessage() << endl;
//     }
//     catch (...)
//     {
//         cout << "Exception during optimization" << endl;
//     }

//     return 0;
// }

// #include <iostream>
// #include <cmath>
// #include <cstdlib>
// #include <ctime>
// #include "gurobi_c++.h"
// #include <vector>

// #include <ctime>
// #include <unistd.h>
// #include <sstream>

// using namespace std;

// // 定义优化目标函数
// void vio_optimization(const std::vector<float> &imu_acc, const std::vector<float> &s, GRBModel &model, float *position, float *velocity)
// {
//     int n = imu_acc.size(); // IMU数据点数
//     float dt = 1;           // 时间步长
//     GRBVar x[n];            // 机器人位姿
//     GRBVar v[n];            // 机器人位姿
//     // std::vector<GRBVar> v;

//     // 定义优化变量属性：第三个参数表示该变量的系数（0表示不参与目标函数的计算），GRB_CONTINUOUS表示该变量为连续型变量
//     for (int i = 0; i < n; i++)
//     {
//         x[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//         v[i] = model.addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS);
//     }
//     std::cout << "0000000000" << std::endl;
//     // 添加约束条件
//     for (int i = 0; i < n - 1; i++)
//     {
//         // 约束机器人位姿的变化模型
//         model.addConstr(x[i + 1] == x[i] + v[i] * dt + 0.5 * dt * dt * imu_acc[i]);
//         model.addConstr(v[i + 1] == v[i] + dt * imu_acc[i]);
//     }
//     std::cout << "1111111111" << std::endl;

//     v[2].set(GRB_DoubleAttr_X, 0.5);

//     GRBQuadExpr obj; // 定义目标函数（最小化机器人位姿和特征点位置的位置估计误差）

//     for (int i = 0; i < n - 1; i++)
//         obj += (x[i] - s[i]) * (x[i] - s[i]);
//     try
//     {
//         model.setObjective(obj);
//         model.optimize();
//         for (int i = 0; i < n; i++)
//             position[i] = x[i].get(GRB_DoubleAttr_X);
//         for (int j = 0; j < n; j++)
//             velocity[j] = v[j].get(GRB_DoubleAttr_X);
//     }
//     catch (GRBException &ex)
//     {
//         cout << "Error code =" << ex.getErrorCode() << endl;
//         cout << ex.getMessage() << endl;
//     }
// }

// /* Facility location:一家公司目前将其产品从5个工厂运送到4个仓库
// 该公司正考虑关闭一些工厂以降低成本。为了减少运输和固定成本，公司应该关闭哪些工厂??
//    Based on an example from Frontline Systems:
//    http://www.solver.com/disfacility.htm
//    Used with permission.
//  */

// int main(int argc, char *argv[])
// {
//     GRBEnv *env = 0;
//     GRBVar *open = 0;
//     GRBVar **transport = 0;
//     int transportCt = 0;
//     try
//     {
//         // Number of plants and 仓库
//         const int nPlants = 5;
//         const int nWarehouses = 4;

//         // 仓库需求以千台计
//         double Demand[] = {15, 18, 14, 20};

//         // Plant capacity in thousands of units
//         double Capacity[] = {20, 22, 17, 19, 18};

//         // Fixed costs for each plant
//         double FixedCosts[] =
//             {12000, 15000, 17000, 13000, 16000};

//         // Transportation costs per thousand units
//         double TransCosts[][nPlants] = {
//             {4000, 2000, 3000, 2500, 4500},
//             {2500, 2600, 3400, 3000, 4000},
//             {1200, 1800, 2600, 4100, 3000},
//             {2200, 2600, 3100, 3700, 3200}};

//         // Model
//         env = new GRBEnv();
//         GRBModel model = GRBModel(*env);
//         model.set(GRB_StringAttr_ModelName, "facility");

//         // Plant open decision variables: open[p] == 1 if plant p is open.
//         open = model.addVars(nPlants, GRB_BINARY);

//         int p;
//         for (p = 0; p < nPlants; ++p)
//         {
//             ostringstream vname;
//             vname << "Open" << p;
//             open[p].set(GRB_DoubleAttr_Obj, FixedCosts[p]);
//             open[p].set(GRB_StringAttr_VarName, vname.str());
//         }

//         // Transportation decision variables: how much to transport from
//         // a plant p to a warehouse w
//         transport = new GRBVar *[nWarehouses];
//         int w;
//         for (w = 0; w < nWarehouses; ++w)
//         {
//             transport[w] = model.addVars(nPlants);
//             transportCt++;

//             for (p = 0; p < nPlants; ++p)
//             {
//                 ostringstream vname;
//                 vname << "Trans" << p << "." << w;
//                 transport[w][p].set(GRB_DoubleAttr_Obj, TransCosts[w][p]);
//                 transport[w][p].set(GRB_StringAttr_VarName, vname.str());
//             }
//         }

//         // The objective is to minimize the total fixed and variable costs
//         model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

//         // Production constraints
//         // Note that the right-hand limit sets the production to zero if
//         // the plant is closed
//         for (p = 0; p < nPlants; ++p)
//         {
//             GRBLinExpr ptot = 0;
//             for (w = 0; w < nWarehouses; ++w)
//             {
//                 ptot += transport[w][p];
//             }
//             ostringstream cname;
//             cname << "Capacity" << p;
//             model.addConstr(ptot <= Capacity[p] * open[p], cname.str());
//         }

//         // Demand constraints
//         for (w = 0; w < nWarehouses; ++w)
//         {
//             GRBLinExpr dtot = 0;
//             for (p = 0; p < nPlants; ++p)
//             {
//                 dtot += transport[w][p];
//             }
//             ostringstream cname;
//             cname << "Demand" << w;
//             model.addConstr(dtot == Demand[w], cname.str());
//         }

//         // Guess at the starting point: close the plant with the highest
//         // fixed costs; open all others

//         // First, open all plants
//         for (p = 0; p < nPlants; ++p)
//         {
//             open[p].set(GRB_DoubleAttr_Start, 1.0);
//         }

//         // Now close the plant with the highest fixed cost
//         cout << "Initial guess:" << endl;
//         double maxFixed = -GRB_INFINITY;
//         for (p = 0; p < nPlants; ++p)
//         {
//             if (FixedCosts[p] > maxFixed)
//             {
//                 maxFixed = FixedCosts[p];
//             }
//         }
//         for (p = 0; p < nPlants; ++p)
//         {
//             if (FixedCosts[p] == maxFixed)
//             {
//                 open[p].set(GRB_DoubleAttr_Start, 0.0);
//                 cout << "Closing plant " << p << endl
//                      << endl;
//                 break;
//             }
//         }

//         // Use barrier to solve root relaxation
//         model.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);

//         // Solve
//         model.optimize();

//         // Print solution
//         cout << "\nTOTAL COSTS: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
//         cout << "SOLUTION:" << endl;
//         for (p = 0; p < nPlants; ++p)
//         {
//             if (open[p].get(GRB_DoubleAttr_X) > 0.99)
//             {
//                 cout << "Plant " << p << " open:" << endl;
//                 for (w = 0; w < nWarehouses; ++w)
//                 {
//                     if (transport[w][p].get(GRB_DoubleAttr_X) > 0.0001)
//                     {
//                         cout << "  Transport " << transport[w][p].get(GRB_DoubleAttr_X) << " units to warehouse " << w << endl;
//                     }
//                 }
//             }
//             else
//             {
//                 cout << "Plant " << p << " closed!" << endl;
//             }
//         }
//     }
//     catch (GRBException e)
//     {
//         cout << "Error code = " << e.getErrorCode() << endl;
//         cout << e.getMessage() << endl;
//     }
//     catch (...)
//     {
//         cout << "Exception during optimization" << endl;
//     }

//     delete[] open;
//     for (int i = 0; i < transportCt; ++i)
//     {
//         delete[] transport[i];
//     }
//     delete[] transport;
//     delete env;
//     return 0;
// }

// /* This example formulates and solves the following simple bilinear model:

//      maximize    x
//      subject to  x + y + z <= 10
//                  x * y <= 2          (bilinear inequality)
//                  x * z + y * z == 1  (bilinear equality)
//                  x, y, z non-negative (x integral in second version)
// */

// // int main(int argc,
// //          char *argv[])
// // {
// //     try
// //     {
// //         GRBEnv env = GRBEnv();

// //         GRBModel model = GRBModel(env);

// //         // Create variables

// //         GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
// //         GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
// //         GRBVar z = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z");

// //         // Set objective
// //         GRBLinExpr obj = x;
// //         model.setObjective(obj, GRB_MAXIMIZE);
// //         // Add linear constraint: x + y + z <= 10
// //         model.addConstr(x + y + z <= 10, "c0");
// //         // Add bilinear inequality constraint: x * y <= 2
// //         model.addQConstr(x * y <= 2, "bilinear0");
// //         // Add bilinear equality constraint: y * z == 1
// //         model.addQConstr(x * z + y * z == 1, "bilinear1");

// //         // set NonConvex to 2
// //         model.set(GRB_IntParam_NonConvex, 2);
// //         model.optimize();

// //         cout << x.get(GRB_StringAttr_VarName) << " "
// //              << x.get(GRB_DoubleAttr_X) << endl;
// //         cout << y.get(GRB_StringAttr_VarName) << " "
// //              << y.get(GRB_DoubleAttr_X) << endl;
// //         cout << z.get(GRB_StringAttr_VarName) << " "
// //              << z.get(GRB_DoubleAttr_X) << endl;

// //         // Constrain x to be integral and solve again
// //         x.set(GRB_CharAttr_VType, GRB_INTEGER);
// //         model.optimize();

// //         cout << x.get(GRB_StringAttr_VarName) << " "
// //              << x.get(GRB_DoubleAttr_X) << endl;
// //         cout << y.get(GRB_StringAttr_VarName) << " "
// //              << y.get(GRB_DoubleAttr_X) << endl;
// //         cout << z.get(GRB_StringAttr_VarName) << " "
// //              << z.get(GRB_DoubleAttr_X) << endl;

// //         cout << "Obj: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
// //     }
// //     catch (GRBException e)
// //     {
// //         cout << "Error code = " << e.getErrorCode() << endl;
// //         cout << e.getMessage() << endl;
// //     }
// //     catch (...)
// //     {
// //         cout << "Exception during optimization" << endl;
// //     }

// //     return 0;
// // }

// // int main()
// // {
// //     srand(time(NULL));
// //     int n = 20; // IMU数据点数
// //     float dt = 1;
// //     float acc = 1;
// //     // 生成IMU加速度测量值
// //     std::vector<float> acc_test(n);
// //     std::vector<float> s(n);
// //     std::vector<float> destina_s(s);
// //     std::vector<float> v(n);
// //     for (int i = 0; i < n - 1; i++)
// //     {
// //         float random = (rand() % 1000);
// //         acc_test[i] = acc + random / 5000;
// //         float random2 = (rand() % 1000);
// //         v[i + 1] = v[i] + dt * acc;
// //         destina_s[i + 1] = destina_s[i] + v[i] * dt + 0.5 * dt * dt * acc;
// //         s[i + 1] = abs(s[i] + v[i] * dt + 0.5 * dt * dt * acc - random2 / 5000);
// //     }

// //     // for (int i = 0; i < n; i++) // 打印结果
// //     //     std::cout << "s[" << i << "] = " << s[i] << std::endl;

// //     // 创建优化模型
// //     GRBEnv env = GRBEnv();
// //     GRBModel model = GRBModel(env);
// //     float position[n];
// //     float velocity[n];
// //     vio_optimization(acc_test, s, model, position, velocity); // 进行优化求解

// //     for (int i = 0; i < n; i++) // 打印结果
// //     {
// //         std::cout << "destina pos[" << i << "] = " << destina_s[i] << "\t \t";
// //         std::cout << "optimize pos = " << position[i] << std::endl;
// //     }
// //     for (int i = 0; i < n; i++) // 打印结果
// //     {
// //         std::cout << "velocity[" << i << "] after optimize = " << velocity[i] << std::endl;
// //     }

// //     return 0;
// // }
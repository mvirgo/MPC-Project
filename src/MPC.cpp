#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// Set the timestep length and duration
// Currently tuned to predict 1 second worth
//设置时间步长长度和持续时间
//目前已调整为预测1秒价值
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
//该值假定使用课堂上呈现的模型。
//
//它是通过测量车辆在行驶过程中形成的半径获得的
//模拟器以恒定的转向角和速度绕着一个圆圈旋转
//地势平坦。
//
//Lf被调谐，直到模拟模型形成半径
//在教室里展示的与之前的半径相符。
//
//这是从前端到具有相似半径的齿轮的长度。
const double Lf = 2.67;

// Set desired speed for the cost function (i.e. max speed)
//设置成本函数的期望速度（即最大速度）
const double ref_v = 120;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
//解算器获取所有状态变量和执行器
//奇异向量中的变量。因此，我们应该建立
//当一个变量开始，另一个变量结束时，我们的生活会变得更轻松。
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  //拟合多项式系数
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implementing MPC below
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;
    //在下面实现MPC
    //`fg`是成本约束的向量，`vars`是可变值的向量（状态和执行器）
    //存储的成本是“fg”的第一个元素。
    //成本的任何增加都应添加到“fg[0]”中。fg[0]=0;

    // Reference State Cost
    // Below defines the cost related the reference state and any anything you think may be beneficial.
    // 
    // Weights for how "important" each cost is - can be tuned

     //参考州成本
     //以下定义了与参考州和
    //任何你认为有益的事情。
    //每个成本的“重要性”权重可以调整
    const int cte_cost_weight = 2000;
    const int epsi_cost_weight = 2000;
    const int v_cost_weight = 1;
    const int delta_cost_weight = 10;
    const int a_cost_weight = 10;
    const int delta_change_cost_weight = 100;
    const int a_change_cost_weight = 10;
    
    // Cost for CTE, psi error and velocity
    // CTE、psi误差和速度的成本
    for (int t = 0; t < N; t++) {
      fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Costs for steering (delta) and acceleration (a)
    // 转向（delta）和加速（a）成本
    for (int t = 0; t < N-1; t++) {
      fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Costs related to the change in steering and acceleration (makes the ride smoother)
    // 与转向和加速变化相关的成本（使行驶更加平稳）
    for (int t = 0; t < N-2; t++) {
      fg[0] += delta_change_cost_weight * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += a_change_cost_weight * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    
    // Setup Model Constraints
    // 设置模型约束
    // Initial constraints
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    //初始约束
    //由于成本位于'fg'的指数0，我们在每个起始指数上加1。
    //这会提升所有其他值的位置。
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    
    // The rest of the constraints
    //其余的限制
    for (int t = 1; t < N; t++) {
      // State at time t + 1
      //在时间t+1时的状态
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      
      // State at time t
      // 在时间t处陈述
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      
      // Actuator constraints at time t only
      //仅在时间t时执行器约束
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*pow(x0,2));
      
      // Setting up the rest of the model constraints
      //设置其余的模型约束
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0-y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//MPC类定义实现。
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  // State vector holds all current values neede for vars below
  //状态向量保存以下变量所需的所有当前值
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Setting the number of model variables (includes both states and inputs).
  // N * state vector size + (N - 1) * 2 actuators (For steering & acceleration)
  //设置模型变量的数量（包括状态和输入）。
//N*状态向量大小+（N-1）*2个执行器（用于转向和加速）
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Setting the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  //自变量的初始值。
//除初始状态外，应为0。
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Sets lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  //设置变量的下限和上限。
//设置所有非执行器上部和下部限位器
//最大正负值。
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  //delta的上限和下限设置为-25和25
  //度（以弧度为单位的值）。
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  
  // Acceleration/decceleration upper and lower limits.
  //加速/减速上限和下限
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  //约束的下限和上限
  //除初始状态外，应为0。
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  
  // Start lower and upper limits at current values
  //从当前值开始设置下限和上限
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  //计算目标和约束的对象

  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  // 注意：你不必担心这些选项
  // options for IPOPT solver
  // IPOPT解算器的选项
  std::string options;
  // Uncomment this if you'd like more print information
  //如果想要更多打印信息，请取消注释
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  //注意：将“稀疏”设置为true可以让解算器充分利用
//对于稀疏例程，这使得计算速度更快。如果你
//可以取消注释其中的1个，看看它是否有区别，但是
//如果取消注释，计算时间将按
//规模。
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  //注意：当前解算器的最大时间限制为0.5秒。
  //你觉得合适的话就换吧。
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  //返回解决方案的位置
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  //解决问题
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  //检查一些解决方案值
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values, along with predicted x and y values to plot in the simulator.
  //返回第一个执行器值，以及要在模拟器中绘制的预测x和y值
  vector<double> solved;
  solved.push_back(solution.x[delta_start]);
  solved.push_back(solution.x[a_start]);
  for (int i = 0; i < N; ++i) {
    solved.push_back(solution.x[x_start + i]);
    solved.push_back(solution.x[y_start + i]);
  }
  
  return solved;

}

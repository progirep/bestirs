/*
 * Simple Vehicle Dynamics
 *
 * File by Ruediger Ehlers
 * Based on a dynamics defined by Matthias Rungger
 */

#include <iostream>
#include <array>

/* SCOTS header */
#include "scots.hh"
/* ode solver */
#include "RungeKutta4.hh"

/* time profiling */
#include "TicToc.hh"
/* memory profiling */
#include <sys/time.h>
#include <sys/resource.h>

/* state space dim */
const int state_dim=3;
/* input space dim */
const int input_dim=2;

/* sampling time */
const double tau = 0.5;

/*
 * data types for the state space elements and input space
 * elements used in uniform grid and ode solvers
 */
using state_type = std::array<double,state_dim>;
using input_type = std::array<double,input_dim>;

/* abbrev of the type for abstract states and inputs */
using abs_type = scots::abs_type;

/* we integrate the vehicle ode by tau sec (the result is stored in x)  */
auto  vehicle_post = [](state_type &x, const input_type &u) {
  /* the ode describing the vehicle */
  auto rhs =[](state_type& xx,  const state_type &x, const input_type &u) {
    double alpha=std::atan(std::tan(u[1])/2.0);
    xx[0] = u[0]*std::cos(alpha+x[2])/std::cos(alpha);
    xx[1] = u[0]*std::sin(alpha+x[2])/std::cos(alpha);
    xx[2] = u[0]*std::tan(u[1]);
  };
  /* simulate (use 10 intermediate steps in the ode solver) */
  scots::runge_kutta_fixed4(rhs,x,u,state_dim,tau,10);
};

/* we integrate the growth bound by 0.3 sec (the result is stored in r)  */
auto radius_post = [](state_type &r, const state_type &, const input_type &u) {
  double c = std::abs(u[0])*std::sqrt(std::tan(u[1])*std::tan(u[1])/4.0+1);
  r[0] = r[0]+c*r[2]*tau;
  r[1] = r[1]+c*r[2]*tau;
};


class UniformGridEx : public scots::UniformGrid {
public:
    template<class grid_point_t>
    UniformGridEx(const int dim,
                const grid_point_t& lb,
                const grid_point_t& ub,
                const grid_point_t& eta) : UniformGrid(dim,lb,ub,eta) {};

    inline std::array<int,3> abstractionStateNumToCellMapper(abs_type id) {
      std::array<int,3> x;
      for(int k = m_dim-1; k > 0; k--) {
        int num=id/m_NN[k];
        id=id%m_NN[k];
        x[k]=num;
      }
      x[0] = id;
      return x;
    }

    inline void calc_nn() {
        UniformGrid::calc_nn();
    }
};

int main(int argv, const char **args) {
  if (argv<2) {
      std::cerr << "Error: Output file name expected!\n";
      return 1;
  }

  /* to measure time */
  TicToc tt;

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  state_type s_lb={{0,0,-3.5}};
  /* upper bounds of the hyper rectangle */
  state_type s_ub={{10,10,3.5}};
  /* grid node distance diameter */
  state_type s_eta={{.2,.2,.2}};
  UniformGridEx ss(state_dim,s_lb,s_ub,s_eta);
  std::cout << "Uniform grid details:" << std::endl;
  ss.print_info();
  
  /* construct grid for the input space */
  /* lower bounds of the hyper rectangle */
  input_type i_lb={{-1,-1}};
  /* upper bounds of the hyper rectangle */
  input_type i_ub={{ 1, 1}};
  /* grid node distance diameter */
  input_type i_eta={{.3,.3}};
  UniformGridEx is(input_dim,i_lb,i_ub,i_eta);
  is.print_info();


  std::cout << "Computing the transition function: " << std::endl;
  /* transition function of symbolic model */
  scots::TransitionFunction tf;
  scots::Abstraction<state_type,input_type> abs(ss,is);

  tt.tic();
  abs.compute_gb(tf, vehicle_post, radius_post);
  tt.toc();

  auto nofGridPoints = is.get_no_gp_per_dim();

  // Compute middle cell for the translation-invariant dimensions
  int middleCell[2];
  middleCell[0] = nofGridPoints[0]/2;
  middleCell[1] = nofGridPoints[1]/2;

  // Print transitions from the middle cell
  is.calc_nn();
  std::ofstream outFile(args[1]);
  std::vector<bool> ifActionsWereUsed;
  for (unsigned int i=0;i<tf.m_no_inputs;i++) ifActionsWereUsed.push_back(false);

  for(abs_type k=0; k<tf.m_no_states; k++) {
      std::array<int,3> postGrid = ss.abstractionStateNumToCellMapper(k);
      //std::cout << "P: " << postGrid[0] << "," << postGrid[1] << "," << postGrid[2] <<  "\n";

      for(abs_type j=0; j<tf.m_no_inputs; j++) {
          for(abs_type v=0; v<tf.m_no_pre[k*tf.m_no_inputs+j]; v++) {
              ifActionsWereUsed[j] = true;
              abs_type pre = tf.m_pre[tf.m_pre_ptr[k*tf.m_no_inputs+j]+v];
              std::array<int,3> preGrid = ss.abstractionStateNumToCellMapper(pre);
              //std::cout << preGrid[0] << "," << preGrid[1] << "," << preGrid[2] << "," << preGrid[3] << "," << middleCell[0] << "," << middleCell[1] << "\n";
              if ((preGrid[0]==middleCell[0]) && (preGrid[1]==middleCell[1])) {
                  outFile << preGrid[0]-middleCell[0] << "," << preGrid[1]-middleCell[1] << "," << preGrid[2] << "," << j << ",";
                  outFile << postGrid[0]-middleCell[0] << "," << postGrid[1]-middleCell[1] << "," << postGrid[2] << "\n";
              }
          }
      }
  }

  // Check if all actions were really used
  for (unsigned int i=0;i<tf.m_no_inputs;i++) {
    if (!ifActionsWereUsed[i]) {
      std::cerr << "Action " << i << "did not occur for any state.\n";
      return 1;
    }
  }



}

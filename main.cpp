#include <iostream>
#include <string>
#include <format>
#include "fuzzy_inference.h"
#include "numcpp.h"
#include <tuple>
#include "Timer.h"
#include "Solver.h"
#define pi 3.14159265358979323846

#define IF(...) (std::tuple(__VA_ARGS__) )
#define THEN(...) (std::tuple(__VA_ARGS__) )
#define RULE(...) (std::tuple(__VA_ARGS__) )
#define MAKE_RULES(...) (std::tuple(__VA_ARGS__) )
#define MAKE_FUZZY_CONTROLLER(x_a,x_b,y_a,y_b,z_a,z_b,num_of_points_for_integrate,num_of_rules,...) (FuzzyController<num_of_rules, decltype(MAKE_RULES(__VA_ARGS__))>(x_a,x_b,y_a,y_b,z_a,z_b,num_of_points_for_integrate,MAKE_RULES(__VA_ARGS__)))


int main()
{   
	const auto theta_PM = TriangleMembershipFunction(2.1, 3.4, 4.7);
    const auto theta_PS = TriangleMembershipFunction(0.5, 1.8, 3.1);
    const auto theta_NM = TriangleMembershipFunction(-4.7, -3.4, -2.1);
    const auto theta_NS = TriangleMembershipFunction(-3.1, -1.8, -0.5);
    const auto theta_ZR = TriangleMembershipFunction(-1.3, 0, 1.3);

    const auto omega_ZR = TriangleMembershipFunction(-1.3, 0, 1.3);
    const auto omega_PS = TriangleMembershipFunction(0.5, 1.8, 3.1);
    const auto omega_NS = TriangleMembershipFunction(-3.1, -1.8, -0.5);

    const auto f_PM = TriangleMembershipFunction(2.1, 3.4, 4.7);
    const auto f_PS = TriangleMembershipFunction(0.5, 1.8, 3.1);
    const auto f_ZR = TriangleMembershipFunction(- 1.3, 0, 1.3);
    const auto f_NM = TriangleMembershipFunction(-4.7, -3.4, -2.1);
    const auto f_NS = TriangleMembershipFunction(-3.1, -1.8, -0.5);

    auto RULE0 = RULE(IF(theta_PM,omega_ZR), THEN(f_PM));
    auto RULE1 = RULE(IF(theta_PS, omega_PS), THEN(f_PS));
    auto RULE2 = RULE(IF(theta_PS, omega_NS), THEN(f_ZR));
    auto RULE3 = RULE(IF(theta_NM, omega_ZR), THEN(f_NM));
    auto RULE4 = RULE(IF(theta_NS, omega_NS), THEN(f_NS));
    auto RULE5 = RULE(IF(theta_NS, omega_PS), THEN(f_ZR));
    auto RULE6 = RULE(IF(theta_ZR, omega_ZR), THEN(f_ZR));
    

    auto Controller = MAKE_FUZZY_CONTROLLER(-4.7,4.7,-3.1,3.1,-4.7,4.7,100,7, RULE0, RULE1, RULE2, RULE3, RULE4, RULE5, RULE6);
    
    //auto SOUT = 

	return 0;	
}

//auto Nx = 1000;
//auto Ny = 1000;
//auto x_vec = np::linspace(-4.7, 4.7, Nx);
//auto y_vec = np::linspace(-4.7, 4.7, Nx);
//
//
//Timer t;
//double tmp;
//for (size_t i = 0; i < Nx; i++)
//{
//    for (size_t j = 0; j < Ny; j++)
//    {
//        tmp = Controller(x_vec[i], y_vec[j]);
//    }
//}
//std::cout << std::format("{}", t.Stop());
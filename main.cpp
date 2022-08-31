#include <iostream>
#include <format>
#include <algorithm>
#include <execution>
#include "Timer.h"
#include "Simulation.h"
#define pi 3.14159265358979323846
#define NUM_OF_THREADS 4
#define	SUPPORT_VALUE_NUM_OF_RULES 49

#define FIND_PARAMS
//#define SIMULATION_OF_FIXED_PARAMS

int main()
{   
	Timer Timer;
	double tau = 0.001; double t_0 = 0.0; double t_end = 10.0;
	double L = 0.5/2.0; double g = 9.81; double m = 0.050; double M = 1.0;

#ifdef FIND_PARAMS
	size_t OmegaNum = 8*NUM_OF_THREADS;
	size_t ForceNum = 20;
	size_t NumOfStartPointsAlongOneAxis = 1;

	double VirtualTime = (t_end - t_0) * OmegaNum * ForceNum * ((std::pow(NumOfStartPointsAlongOneAxis, 2) - NumOfStartPointsAlongOneAxis) / 2 + NumOfStartPointsAlongOneAxis);
	std::cout << std::format("\n\tthe program will be executed for about {:.5f} s\n", VirtualTime / 550.0 / ((double)NUM_OF_THREADS)*(SUPPORT_VALUE_NUM_OF_RULES/7));
	double ThetaMax = 0.2617993877991494;

	double Omega_a = 3;
	double Omega_b = 6;
	double Force_a = 3;
	double Force_b = 10;

	auto ExecuteSimulateALotFunction = [=](size_t i)->void
	{
		size_t chunck_size = OmegaNum / NUM_OF_THREADS;
		double step = (Omega_b - Omega_a) / (OmegaNum - 1);
		double offset = i * chunck_size * step;
		auto OmegaVec_i = np::linspace(Omega_a + offset, Omega_a + offset + (chunck_size-1) * step, chunck_size);
		auto ForceVec_i = np::linspace(Force_a, Force_b, ForceNum);
		SimulateALot(
			ThetaMax,
			OmegaVec_i, ForceVec_i,
			NumOfStartPointsAlongOneAxis,
			tau, t_0, t_end,
			L, g, m, M,
			{ -pi / 2,pi / 2 }
		);
	};
	std::array<size_t, NUM_OF_THREADS> ArrayForDistributionOfTaskByIndecies;
	for (size_t i = 0; i < NUM_OF_THREADS; i++)
	{
		ArrayForDistributionOfTaskByIndecies[i] = i;
	}
	std::cout << std::format("\tstd::for_each with par_unseq\n");
	std::for_each(std::execution::par_unseq,
		ArrayForDistributionOfTaskByIndecies.begin(), ArrayForDistributionOfTaskByIndecies.end(), 
		ExecuteSimulateALotFunction
		);

	std::cout << std::format("\tPhysicsParams:\n");
	std::cout << std::format("\t\ttau={} s\n",tau);
	std::cout << std::format("\t\tT={} s\n", t_end-t_0);
	std::cout << std::format("\t\tL={} cm\n",2*L*100.0);
	std::cout << std::format("\t\tm={} g\n", m*1000.0);
	std::cout << std::format("\t\tm={} kg\n", M);
	std::cout << std::format("\tThetaMax={} rad\n", ThetaMax);
	std::cout << std::format("\tOmegaGrid.size={}\n", OmegaNum);
	std::cout << std::format("\tForceGrid.size={}\n", ForceNum);
	std::cout << std::format("\tnum of simulations in one point of grid={}\n", NumOfStartPointsAlongOneAxis);
	std::cout << std::format("\ttotal simulation time={} s\n", VirtualTime);
#endif

#ifdef SIMULATION_OF_FIXED_PARAMS
	size_t NumOfStartPointsAlongOneAxis = 10;
	double VirtualTime = (t_end - t_0) * ((std::pow(NumOfStartPointsAlongOneAxis, 2) - NumOfStartPointsAlongOneAxis) / 2 + NumOfStartPointsAlongOneAxis-1);
	std::cout << std::format("\n\tthe program will be executed for about {:.5f} s\n", VirtualTime / 550.0 );
	double ThetaMax = 0.2617993877991494;
	double OmegaMax = 3;
	double ForceMax = 10;
	
	SimulateWithOneController(ThetaMax, OmegaMax, ForceMax, NumOfStartPointsAlongOneAxis,
		tau, t_0, t_end,
		L, g, m, M,
		{ -ThetaMax,ThetaMax }
		);


#endif

	std::cout << std::format("\tprogram execution time ={} s\n", Timer.Stop());
	return 0;	
}
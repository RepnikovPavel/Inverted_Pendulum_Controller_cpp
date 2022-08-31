#pragma once

#include <vector>
#include <map>
#include <algorithm>

#include "FuzzyInference.h"
#include "SystemOfUnitsTranlation.h"
#include "Solver.h"
#include "NumCPP.h"


void PrintSimulateALotResults(std::pair<double, std::array<double, 2>>);

void SimulateALot(
            const double ThetaMax, const std::vector<double> OmegaVec, const std::vector<double> ForceVec,
            size_t N,
            double tau, double t_0, double t_end,
            double L, double g, double m, double M,
            std::array<double, 2> condition_of_break
            )
{

    std::map<double,std::array<double,2>> SimulResultsForSuccessfulAttempts;

    auto X0_Voltage_Grid = np::linspace(-0.5, 0.5, N);
    auto X1_Voltage_Grid = np::linspace(-0.5, 0.5, N);


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
    const auto f_ZR = TriangleMembershipFunction(-1.3, 0, 1.3);
    const auto f_NM = TriangleMembershipFunction(-4.7, -3.4, -2.1);
    const auto f_NS = TriangleMembershipFunction(-3.1, -1.8, -0.5);

    auto RULE0 = RULE(IF(theta_PM, omega_ZR), THEN(f_PM));
    auto RULE1 = RULE(IF(theta_PS, omega_PS), THEN(f_PS));
    auto RULE2 = RULE(IF(theta_PS, omega_NS), THEN(f_ZR));
    auto RULE3 = RULE(IF(theta_NM, omega_ZR), THEN(f_NM));
    auto RULE4 = RULE(IF(theta_NS, omega_NS), THEN(f_NS));
    auto RULE5 = RULE(IF(theta_NS, omega_PS), THEN(f_ZR));
    auto RULE6 = RULE(IF(theta_ZR, omega_ZR), THEN(f_ZR));

    double signal_value = 5.0;

    auto Controller = MAKE_FUZZY_CONTROLLER(-4.7, 4.7, -3.1, 3.1, -4.7, 4.7,signal_value, 100, 7, RULE0, RULE1, RULE2, RULE3, RULE4, RULE5, RULE6);

    for (size_t i = 0; i < OmegaVec.size(); i++)
    {
        for (size_t j = 0; j < ForceVec.size(); j++)
        {

            double theta_max_SI = ThetaMax;
            double omega_max_SI = OmegaVec[i];
            double F_max_SI = ForceVec[j];

            auto X0_Translator = SOUTransfer::Linear(-theta_max_SI, theta_max_SI, -4.7, 4.7);
            auto X1_Translator = SOUTransfer::Linear(-omega_max_SI, omega_max_SI, -3.1, 3.1);
            auto X2_Translator = SOUTransfer::Linear(-F_max_SI, F_max_SI, -4.7, 4.7);

            double b = (3 * m) / (7 * (M + m));

            auto Simulation = CreateSimulation(
                Controller, X0_Translator, X1_Translator, X2_Translator,
                tau, t_0, t_end, L, g, b, m, M,
                condition_of_break);

            bool IsControllerGood = true;
            double MaxDistance = 0.0;
            for (size_t k1 = 0; k1 < N; k1++)
            {
                if (IsControllerGood==false)
                {
                    break;
                }
                for (size_t k2 = 0; k2 < N-k1; k2++)
                {
                    auto X0_SI_Value = X0_Translator.inverse_call(X0_Voltage_Grid[k1]);
                    auto X1_SI_Value = X1_Translator.inverse_call(X1_Voltage_Grid[k2]);
                    if (X0_SI_Value==0.0 && X1_SI_Value ==0.0)
                    {
                        continue;
                    }
                    Simulation.Set_Y_0(
                        { X0_SI_Value,
                          X1_SI_Value,
                        0.0,0.0 });
                    Simulation.Run();
                    auto SimResults = Simulation.GetSimResults();
                    if (std::get<2>(SimResults) == 1)
                    {
                        IsControllerGood = false;
                        break;
                    }
                    MaxDistance = std::max(MaxDistance, std::abs(std::get<0>(SimResults)[2]));
                }
            }
            if (IsControllerGood)
            {
                SimulResultsForSuccessfulAttempts[MaxDistance] = { OmegaVec[i] , ForceVec[j] };
            }
        }
    }
    auto Results =*(std::min_element(SimulResultsForSuccessfulAttempts.begin(), SimulResultsForSuccessfulAttempts.end()));

    std::cout << std::format("\tOmegaMax={} rad/s;ForceMax={} N,|y(t_end)|={} cm\n", std::get<1>(Results)[0], std::get<1>(Results)[1], 100.0 * std::get<0>(Results));
}

#undef IF
#undef THEN
#undef RULE
#undef MAKE_RULES
#undef MAKE_FUZZY_CONTROLLER
#undef GetControllerType
#undef CreateSimulation
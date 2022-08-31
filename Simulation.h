#pragma once

#include <vector>
#include <map>
#include <algorithm>
#include <fstream>
#include <string>

#include "SystemOfUnitsTranlation.h"
#include "Solver.h"
#include "NumCPP.h"
#include "COntrollerApi.h"
#include "Paths.h"


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

    auto Controller = CR_API::GetController();

    double _a_x = Controller.Get_a_x();
    double _b_x = Controller.Get_b_x();
    double _a_y = Controller.Get_a_y();
    double _b_y = Controller.Get_b_y();
    double _a_z = Controller.Get_a_z();
    double _b_z = Controller.Get_b_z();
    
    auto X0_Voltage_Grid = np::linspace(_a_x/10, _b_x/10, N);
    auto X1_Voltage_Grid = np::linspace(_a_y/10,_b_y/10, N);

    for (size_t i = 0; i < OmegaVec.size(); i++)
    {
        for (size_t j = 0; j < ForceVec.size(); j++)
        {

            double theta_max_SI = ThetaMax;
            double omega_max_SI = OmegaVec[i];
            double F_max_SI = ForceVec[j];

            auto X0_Translator = SOUTransfer::Linear(-theta_max_SI, theta_max_SI, _a_x, _b_x);
            auto X1_Translator = SOUTransfer::Linear(-omega_max_SI, omega_max_SI, _a_y, _b_y);
            auto X2_Translator = SOUTransfer::Linear(-F_max_SI, F_max_SI, _a_z, _b_z);

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
    if (!SimulResultsForSuccessfulAttempts.empty())
    {
        auto Results = *(std::min_element(SimulResultsForSuccessfulAttempts.begin(), SimulResultsForSuccessfulAttempts.end()));

        std::cout << std::format("\tOmegaMax={} rad/s;ForceMax={} N,|y(t_end)|={} cm\n", std::get<1>(Results)[0], std::get<1>(Results)[1], 100.0 * std::get<0>(Results));
    }

}

void SimulateWithOneController(const double ThetaMax, const double OmegaMax, const double ForceMax,
    const size_t N,
    double tau, double t_0, double t_end,
    double L, double g, double m, double M,
    std::array<double, 2> condition_of_break)
{
    auto Controller = CR_API::GetController();
    double _a_x = Controller.Get_a_x();
    double _b_x = Controller.Get_b_x();
    double _a_y = Controller.Get_a_y();
    double _b_y = Controller.Get_b_y();
    double _a_z = Controller.Get_a_z();
    double _b_z = Controller.Get_b_z();

    double theta_max_SI = ThetaMax;
    double omega_max_SI = OmegaMax;
    double F_max_SI = ForceMax;

    auto X0_Translator = SOUTransfer::Linear(-theta_max_SI, theta_max_SI, _a_x, _b_x);
    auto X1_Translator = SOUTransfer::Linear(-omega_max_SI, omega_max_SI, _a_y, _b_y);
    auto X2_Translator = SOUTransfer::Linear(-F_max_SI, F_max_SI, _a_z, _b_z);

    double b = (3 * m) / (7 * (M + m));

    auto Simulation = CreateSimulation(
        Controller, X0_Translator, X1_Translator, X2_Translator,
        tau, t_0, t_end, L, g, b, m, M,
        condition_of_break);
    
    auto X0_Voltage_Grid = np::linspace(_a_x *9/ 10, _b_x *9/ 10, N);
    auto X1_Voltage_Grid = np::linspace(_a_y *9/ 10, _b_y *9/ 10, N);


  
    double MaxDistance = 0.0;
    std::vector<std::vector<std::array<double, 4>>> Trajectories;
    std::vector<size_t> CodesOfSim;
    for (size_t k1 = 0; k1 < N; k1++)
    {
        for (size_t k2 = 0; k2 < N - k1; k2++)
        {
            auto X0_SI_Value = X0_Translator.inverse_call(X0_Voltage_Grid[k1]);
            auto X1_SI_Value = X1_Translator.inverse_call(X1_Voltage_Grid[k2]);
            if (X0_SI_Value == 0.0 && X1_SI_Value == 0.0)
            {
                continue;
            }
            Simulation.Set_Y_0(
                { X0_SI_Value,
                  X1_SI_Value,
                0.0,0.0 });
            auto Trajectory = Simulation.RunWithTrajectoryRecording();
            Trajectories.push_back(Trajectory);
            auto SimResults = Simulation.GetSimResults();
            CodesOfSim.push_back(std::get<2>(SimResults));
            MaxDistance = std::max(MaxDistance, std::abs(std::get<0>(SimResults)[2]));
        }
    }
    // calling python to plot Trajectories
    std::ofstream TrsFile(PATH_TRAJECTORIES, std::ios::trunc);

    for (size_t i = 0; i < Trajectories.size(); i++)
    {
        //code of sim writing
        TrsFile << CodesOfSim[i] << '\n';

        //x writing
        for (size_t j = 0; j < Trajectories[i].size(); j++)
        {
            TrsFile << X0_Translator.call(Trajectories[i][j][0]) << ' ';
        }
        TrsFile << '\n';
        //y writing
        for (size_t j = 0; j < Trajectories[i].size(); j++)
        {
            TrsFile << X1_Translator.call(Trajectories[i][j][1]) << ' ';
        }
        TrsFile << '\n';
    }

    TrsFile.close();
    std::ofstream PythonArgs(PATH_PYTHON_ARGS, std::ios::trunc);
    PythonArgs << std::string(PATH_TRAJECTORIES);
    PythonArgs.close();

    std::string cmd_command;
    cmd_command += PATH_PYTHON_EXE;
    cmd_command += " ";
    cmd_command += PATH_PYTHON_PY_FILE;

    system(cmd_command.c_str());
}



#undef CreateSimulation

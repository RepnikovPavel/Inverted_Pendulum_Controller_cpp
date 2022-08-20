#pragma once
#include <array>

template<
    typename ControllerType,
    typename X0_Translator_type,
    typename X1_Translator_type,
    typename X2_Translator_type>
class ODESolver 
{
public:
    ODESolver(
        ControllerType _Controller,
        X0_Translator_type _X0_Translator,
        X1_Translator_type _X1_Translator,
        X2_Translator_type _X2_Translator,
        double _tau, double _t_0, double _t_end,
        double _L, double _g, double _b, double _m, double _M,
        std::array<double, 2> _condition_of_break,
        std::array<double, 4> _Y_0
    ) 
    {
        Controller = _Controller;
        X0_Translator = _X0_Translator;
        X1_Translator = _X1_Translator;
        X2_Translator = _X2_Translator;
        tau = _tau;t_0 = _t_0;t_end = _t_end;
        condition_of_break = _condition_of_break;
        L = _L; g = _g; b = _b; m = _m; M = _M;

        Y_vec_buffer = _Y_0;
        num_of_points_in_solved_vec = (unsigned int)((t_end - t_0) / tau);
    }
    ODESolver(const ODESolver& other)
    {
        Controller = other.Controller;
        X0_Translator = other.X0_Translator;
        X1_Translator = other.X1_Translator;
        X2_Translator = other.X2_Translator;
        tau = other.tau; t_0 = other.t_0; t_end = other.t_end;
        condition_of_break = other.condition_of_break;
        L = other.L; g = other.g; b = other.b; m = other.m; M = other.M;

        Y_vec_buffer = other.Y_0;
        num_of_points_in_solved_vec = other.num_of_points_in_solved_vec;
    }
    ODESolver& operator=(const ODESolver& other)
    {
        Controller = other.Controller;
        X0_Translator = other.X0_Translator;
        X1_Translator = other.X1_Translator;
        X2_Translator = other.X2_Translator;
        tau = other.tau; t_0 = other.t_0; t_end = other.t_end;
        condition_of_break = other.condition_of_break;
        L = other.L; g = other.g; b = other.b; m = other.m; M = other.M;

        Y_vec_buffer = other.Y_0;
        num_of_points_in_solved_vec = other.num_of_points_in_solved_vec;
        return *this;
    }

    void Set_Y_0(std::array<double, 4> _Y_0) 
    {
        Y_vec_buffer = _Y_0;
    }

    void Run() 
    {
        unsigned int code_of_sim = 0;
        double time_of_sim = 0.0;
        if (condition_of_break[0] > Y_vec_buffer[0] || condition_of_break[1] < Y_vec_buffer[0])
        {
            code_of_sim = 1;
            SimResult = std::make_tuple(Y_vec_buffer, time_of_sim, code_of_sim);
            return;
        }
        double Force_t_n = 0.0;
        NextPoint(Force_t_n);
        for (size_t i = 1; i < num_of_points_in_solved_vec - 1; i++)
        {
            if (condition_of_break[0] > Y_vec_buffer[0] || condition_of_break[1] < Y_vec_buffer[0])
            {
                code_of_sim = 1;
                time_of_sim = i * tau;
                SimResult = std::make_tuple(Y_vec_buffer, time_of_sim, code_of_sim);
                return;
            }
            Force_t_n = Controller(X0_Translator.call(Y_vec_buffer[0]), X1_Translator.call(Y_vec_buffer[1]));

            if (Force_t_n == -5.0)
            {
                Force_t_n = 0.0;
            }
            NextPoint(X2_Translator.inverse_call(Force_t_n));
        }
        time_of_sim = t_end - t_0;
        SimResult = std::make_tuple(Y_vec_buffer, time_of_sim, code_of_sim);
        return;
    
    }

private:
    void NextPoint(double _f)
    {
        Modify_F_vec(_f);
        Modify_Y_vec_buffer();
    }

    void Modify_F_vec(double f)
    {
        auto theta = Y_vec_buffer[0];
        auto omega = Y_vec_buffer[1];
        auto v = Y_vec_buffer[3];

        F_vec_buffer[0] = omega;
        F_vec_buffer[1] = 1 / (1 - b * std::pow(std::cos(theta), 2)) * (
            3 * g / (7 * L) * std::sin(theta) - b * f / (m * L) * std::cos(theta) - b * std::sin(theta) * std::cos(
                theta) * std::pow(omega, 2));
        F_vec_buffer[2] = v;
        F_vec_buffer[3] = 1 / (1 - b * std::pow(std::cos(theta), 2)) * (
            f / (M + m) - b * g * std::sin(theta) * std::cos(theta) + 7 / 3 * std::pow(omega, 2) * b * L * std::sin(theta));

    }
    void Modify_Y_vec_buffer()
    {
        for (size_t i = 0; i < 4; i++)
        {
            Y_vec_buffer[i] += tau * F_vec_buffer[i];
        }
    }
public:
    std::tuple<std::array<double, 4>, double, unsigned int> GetSimResults() 
    {
        return SimResult;
    }
    void PrintSimResult()
    {
#define local_pi 3.14159265358979323846
        std::cout << std::format("theta_end={} deg\n", std::get<0>(SimResult)[0] / local_pi * 180.0);
        std::cout << std::format("omega_end={} deg/s\n", std::get<0>(SimResult)[1] / local_pi * 180.0);
        std::cout << std::format("y_end={} cm\n", -std::get<0>(SimResult)[2] * 100.0);
        std::cout << std::format("v_end={} cm/s\n", -std::get<0>(SimResult)[3] * 100.0);
        std::cout << std::format("time_of_sim={} s\n", std::get<1>(SimResult));
        std::cout << std::format("has the pendulum fallen? {}\n", (std::get<1>(SimResult) == 0 ? false : true));
#undef local_pi
    }

private:
    ControllerType Controller;
    X0_Translator_type X0_Translator;
    X1_Translator_type X1_Translator;
    X2_Translator_type X2_Translator;
    double tau; double t_0; double t_end;
    std::array<double, 2> condition_of_break;
    double L; double g; double b; double m; double M;
    
    std::array<double, 4> F_vec_buffer;
    std::array<double, 4> Y_vec_buffer;
    unsigned int num_of_points_in_solved_vec;

    std::tuple<std::array<double, 4>, double, unsigned int> SimResult;
};

#define CreateSimulation(Controller, X0_Translator, X1_Translator, X2_Translator,\
tau, t_0, t_end, L, g, b, m, M,\
condition_of_break, Y_0)\
( ODESolver<decltype(Controller),decltype(X0_Translator),decltype(X1_Translator),decltype(X2_Translator)>(Controller, X0_Translator, X1_Translator, X2_Translator,tau, t_0, t_end, L, g, b, m, M,condition_of_break, Y_0)  )


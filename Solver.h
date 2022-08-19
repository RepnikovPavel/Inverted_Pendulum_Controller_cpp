#pragma once
#include <array>

std::array<double, 4>& F_vec(
    std::array<double,4>& F_vec_buffer, std::array<double, 4>& Y_vec_t_n,
	double L, double g, double f, double b, double m, double M)
{
    auto theta = Y_vec_t_n[0];
    auto omega = Y_vec_t_n[1];
    auto y = Y_vec_t_n[2];
    auto v = Y_vec_t_n[3];
    
    F_vec_buffer[0] = omega;
    F_vec_buffer[1] = 1 / (1 - b * std::pow(std::cos(theta),2)) * (
        3 * g / (7 * L) * std::sin(theta) - b * f / (m * L) * std::cos(theta) - b * std::sin(theta) * std::cos(
            theta) * std::pow(omega,2));
    F_vec_buffer[2] = v;
    F_vec_buffer[3] = 1 / (1 - b * std::pow(std::cos(theta),2)) * (
        f / (M + m) - b * g * std::sin(theta) * std::cos(theta) + 7 / 3 * std::pow(omega,2) * b * L * std::sin(theta));
    return F_vec_buffer;
}

std::array<double, 4>& NextPoint(
    double tau, std::array<double, 4>& F_vec_buffer, std::array<double, 4>& Y_vec_t_n,
    double L, double g, double f, double b, double m, double M)
{
    auto F = F_vec(F_vec_buffer, Y_vec_t_n, L, g, f, b, m, M);
    for (size_t i = 0; i < 4; i++)
    {
        Y_vec_t_n[i] += tau * F[i];
    }
    return Y_vec_t_n;
}

template<
    typename ControllerType,
    typename SystemOfUnitsTranlationType>
std::tuple<std::array<double, 4>, double, unsigned int> SolveODE(
    double tau, double t_0, double t_end,
    std::array<double, 4> Y_0, std::array<double, 2> condition_of_break,
    double L, double g, double f, double b, double m, double M,
    ControllerType Controller,
    SystemOfUnitsTranlationType SOUT
     ) 
{
    unsigned int code_of_sim = 0;
    double time_of_sim = 0.0;
    if (condition_of_break[0]>Y_0[0] || condition_of_break[1]<Y_0[0])
    {
        code_of_sim = 1;
        return std::make_tuple(Y_0, time_of_sim, code_of_sim);
    }
    std::array<double, 4> F_vec_buffer{0.0};
    std::array<double, 4> Y_vec_buffer(Y_0);
    unsigned int num_of_points_in_solved_vec = (t_end - t_0) / tau;
    double Force_t_n = 0.0;
    Y_vec_buffer = NextPoint(tau, F_vec_buffer, Y_vec_buffer, L, g, f, b, m, M);
    for (size_t i = 1; i < num_of_points_in_solved_vec-1; i++)
    {
        if (condition_of_break[0] > Y_vec_buffer[0] || condition_of_break[1] < Y_vec_buffer[0])
        {
            code_of_sim = 1;
            time_of_sim = i * tau;
            return std::make_tuple(Y_vec_buffer, time_of_sim, code_of_sim);
        }
        /// тут нужно перевести из одной системы единиц в другую(Из СИ в вольты)

        Force_t_n = Controller(Y_vec_buffer[0], Y_vec_buffer[1]);

        if (Force_t_n==-5.0)
        {
            Force_t_n = 0.0;
        }
        /// Тут нужно перевести Force_t_n обратно в СИ
        Y_vec_buffer = NextPoint(tau, F_vec_buffer, Y_vec_buffer, L, g, f, b, m, M);
    }
    return std::make_tuple(Y_vec_buffer, time_of_sim, code_of_sim);

}
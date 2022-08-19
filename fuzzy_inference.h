#pragma once
#include<array>
#include<algorithm>
#define GET(tuple_obj,index) (std::get<index>(tuple_obj))

class TriangleMembershipFunction 
{
    double l_point;
    double m_point; 
    double r_point;
public:
    TriangleMembershipFunction(){};
    TriangleMembershipFunction(double l_point, double m_point, double r_point)
    {
        this->l_point = l_point;
        this->m_point = m_point;
        this->r_point = r_point;
    }
    double operator() (double x)
    {
        if (x <= l_point) { return 0.0; }
        else if (x >= r_point) { return 0.0; }
        else if (l_point < x && x <= m_point) { return 1.0 / (m_point - l_point) * x + l_point / (l_point - m_point); }
        else { return 1.0 / (m_point - r_point) * x + r_point / (r_point - m_point); }
    }
    TriangleMembershipFunction& operator=(const TriangleMembershipFunction& other) 
    {
        l_point = other.l_point;
        m_point = other.m_point;
        r_point = other.r_point;
        return *this;
    }
    ~TriangleMembershipFunction() {};

};


template<std::size_t N_r,typename RULES>
class FuzzyController
{   
    std::array<double,N_r> AlphaBuffer;
    std::array<double, N_r> MaxBuffer;
    RULES Rules;
    std::size_t N;
    double x_a;
    double x_b;
    double y_a;
    double y_b;
    double z_a;
    double z_b;
public:
  
    FuzzyController(double x_a, double x_b, double y_a, double y_b, double z_a, double z_b,std::size_t N,RULES Rules)
    {   
        this->x_a = x_a;
        this->x_b = x_b;
        this->y_a = y_a;
        this->y_b = y_b;
        this->z_a = z_a;
        this->z_b = z_b;
        this->N = N;
        this->Rules = Rules;
    }
    double operator() (double x_0,double x_1) 
    {
        if (x_a <= x_0 && x_0 <= x_b) { ; }
        else if (x_0 > x_b){x_0 -= x_b - x_a; }
        else { x_0 += x_b - x_a; }

        if (y_a <= x_1 && x_1 <= y_b) { ; }
        else 
        {
            double dist1 = std::abs(x_1 - y_a);
            double dist2= std::abs(x_1 - y_b);
            return dist1 > dist2 ? dist2 : dist1;
        }
        

        AlphaBuffer[0] = std::min(GET(GET(GET(Rules, 0), 0), 0)(x_0), GET(GET(GET(Rules, 0), 0), 1)(x_1));
        AlphaBuffer[1] = std::min(GET(GET(GET(Rules, 1), 0), 0)(x_0), GET(GET(GET(Rules, 1), 0), 1)(x_1));
        AlphaBuffer[2] = std::min(GET(GET(GET(Rules, 2), 0), 0)(x_0), GET(GET(GET(Rules, 2), 0), 1)(x_1));
        AlphaBuffer[3] = std::min(GET(GET(GET(Rules, 3), 0), 0)(x_0), GET(GET(GET(Rules, 3), 0), 1)(x_1));
        AlphaBuffer[4] = std::min(GET(GET(GET(Rules, 4), 0), 0)(x_0), GET(GET(GET(Rules, 4), 0), 1)(x_1));
        AlphaBuffer[5] = std::min(GET(GET(GET(Rules, 5), 0), 0)(x_0), GET(GET(GET(Rules, 5), 0), 1)(x_1));
        AlphaBuffer[6] = std::min(GET(GET(GET(Rules, 6), 0), 0)(x_0), GET(GET(GET(Rules, 6), 0), 1)(x_1));

       
        double numerator = 0.0;
        double denominator = 0.0;
        double h = (z_b - z_a) / N;
        for (size_t i = 0; i < N; i++)
        {
            MaxBuffer[0] = std::min(AlphaBuffer[0], GET(GET(GET(Rules, 0), 1), 0)(z_a + h * i));
            MaxBuffer[1] = std::min(AlphaBuffer[1], GET(GET(GET(Rules, 1), 1), 0)(z_a + h * i));
            MaxBuffer[2] = std::min(AlphaBuffer[2], GET(GET(GET(Rules, 2), 1), 0)(z_a + h * i));
            MaxBuffer[3] = std::min(AlphaBuffer[3], GET(GET(GET(Rules, 3), 1), 0)(z_a + h * i));
            MaxBuffer[4] = std::min(AlphaBuffer[4], GET(GET(GET(Rules, 4), 1), 0)(z_a + h * i));
            MaxBuffer[5] = std::min(AlphaBuffer[5], GET(GET(GET(Rules, 5), 1), 0)(z_a + h * i));
            MaxBuffer[6] = std::min(AlphaBuffer[6], GET(GET(GET(Rules, 6), 1), 0)(z_a + h * i));
            double ro_dV =  h*(* (std::max_element(MaxBuffer.begin(), MaxBuffer.end())));
            numerator += ro_dV * (z_a + h * i);
            denominator += ro_dV;
        }
        if (numerator==0.0)
        {
            return -5.0;
        }
        return numerator/denominator;
    }
	

};
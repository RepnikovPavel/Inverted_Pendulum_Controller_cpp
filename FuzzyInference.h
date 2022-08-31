#pragma once
#include<array>
#include<algorithm>

class TriangleMembershipFunction 
{
public:
    TriangleMembershipFunction() {}
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
    ~TriangleMembershipFunction() {}
private:
    double l_point;
    double m_point;
    double r_point;
};


template<std::size_t N_r,typename RULES>
class FuzzyController
{   

public:
    FuzzyController() {};
    FuzzyController(double _x_a, double _x_b, double _y_a, double _y_b, double _z_a, double _z_b,double _signal_value,std::size_t _N,RULES _Rules)
    {   
        x_a = _x_a;
        x_b = _x_b;
        y_a = _y_a;
        y_b = _y_b;
        z_a = _z_a;
        z_b = _z_b;
        signal_value = _signal_value;
        N = _N;
        Rules = _Rules;
    }
    FuzzyController(const FuzzyController& other)
    {
        x_a = other.x_a;
        x_b = other.x_b;
        y_a = other.y_a;
        y_b = other.y_b;
        z_a = other.z_a;
        z_b = other.z_b;
        signal_value = other.signal_value;
        N = other.N;
        Rules = other.Rules;
    }
    FuzzyController& operator=(const FuzzyController& other)
    {
        x_a = other.x_a;
        x_b = other.x_b;
        y_a = other.y_a;
        y_b = other.y_b;
        z_a = other.z_a;
        z_b = other.z_b;
        signal_value = other.signal_value;
        N = other.N;
        Rules = other.Rules;
        return *this;
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
        
#define GET(tuple_obj,index) (std::get<index>(tuple_obj))
        AlphaBuffer[0] = std::min(GET(GET(GET(Rules, 0), 0), 0)(x_0), GET(GET(GET(Rules, 0), 0), 1)(x_1));
        AlphaBuffer[1] = std::min(GET(GET(GET(Rules, 1), 0), 0)(x_0), GET(GET(GET(Rules, 1), 0), 1)(x_1));
        AlphaBuffer[2] = std::min(GET(GET(GET(Rules, 2), 0), 0)(x_0), GET(GET(GET(Rules, 2), 0), 1)(x_1));
        AlphaBuffer[3] = std::min(GET(GET(GET(Rules, 3), 0), 0)(x_0), GET(GET(GET(Rules, 3), 0), 1)(x_1));
        AlphaBuffer[4] = std::min(GET(GET(GET(Rules, 4), 0), 0)(x_0), GET(GET(GET(Rules, 4), 0), 1)(x_1));
        AlphaBuffer[5] = std::min(GET(GET(GET(Rules, 5), 0), 0)(x_0), GET(GET(GET(Rules, 5), 0), 1)(x_1));
        AlphaBuffer[6] = std::min(GET(GET(GET(Rules, 6), 0), 0)(x_0), GET(GET(GET(Rules, 6), 0), 1)(x_1));
#undef GET
       
        double numerator = 0.0;
        double denominator = 0.0;
        double h = (z_b - z_a) / N;
        for (size_t i = 0; i < N; i++)
        {
#define GET(tuple_obj,index) (std::get<index>(tuple_obj))
            MaxBuffer[0] = std::min(AlphaBuffer[0], GET(GET(GET(Rules, 0), 1), 0)(z_a + h * i));
            MaxBuffer[1] = std::min(AlphaBuffer[1], GET(GET(GET(Rules, 1), 1), 0)(z_a + h * i));
            MaxBuffer[2] = std::min(AlphaBuffer[2], GET(GET(GET(Rules, 2), 1), 0)(z_a + h * i));
            MaxBuffer[3] = std::min(AlphaBuffer[3], GET(GET(GET(Rules, 3), 1), 0)(z_a + h * i));
            MaxBuffer[4] = std::min(AlphaBuffer[4], GET(GET(GET(Rules, 4), 1), 0)(z_a + h * i));
            MaxBuffer[5] = std::min(AlphaBuffer[5], GET(GET(GET(Rules, 5), 1), 0)(z_a + h * i));
            MaxBuffer[6] = std::min(AlphaBuffer[6], GET(GET(GET(Rules, 6), 1), 0)(z_a + h * i));
#undef GET
            double ro_dV =  h*(* (std::max_element(MaxBuffer.begin(), MaxBuffer.end())));
            numerator += ro_dV * (z_a + h * i);
            denominator += ro_dV;
        }

        if (numerator==0.0)
        {
            return signal_value;
        }
        return numerator/denominator;
    }
    double GetSingalValue() 
    {
        return signal_value;
    }
private:
    std::array<double, N_r> AlphaBuffer;
    std::array<double, N_r> MaxBuffer;
    RULES Rules;
    std::size_t N;
    double x_a;
    double x_b;
    double y_a;
    double y_b;
    double z_a;
    double z_b;
    double signal_value;
};

#define IF(...) (std::tuple(__VA_ARGS__) )
#define THEN(...) (std::tuple(__VA_ARGS__) )
#define RULE(...) (std::tuple(__VA_ARGS__) )
#define MAKE_RULES(...) (std::tuple(__VA_ARGS__) )
#define MAKE_FUZZY_CONTROLLER(x_a,x_b,y_a,y_b,z_a,z_b,signal_value,num_of_points_for_integrate,num_of_rules,...) (FuzzyController<num_of_rules, decltype(MAKE_RULES(__VA_ARGS__))>(x_a,x_b,y_a,y_b,z_a,z_b,signal_value,num_of_points_for_integrate,MAKE_RULES(__VA_ARGS__)))
#define GetControllerType(Controller) decltype(Controller)
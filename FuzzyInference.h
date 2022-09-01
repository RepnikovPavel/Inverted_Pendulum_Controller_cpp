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
        AlphaBuffer[7] = std::min(GET(GET(GET(Rules, 7), 0), 0)(x_0), GET(GET(GET(Rules, 7), 0), 1)(x_1));
        AlphaBuffer[8] = std::min(GET(GET(GET(Rules, 8), 0), 0)(x_0), GET(GET(GET(Rules, 8), 0), 1)(x_1));
        AlphaBuffer[9] = std::min(GET(GET(GET(Rules, 9), 0), 0)(x_0), GET(GET(GET(Rules, 9), 0), 1)(x_1));
        AlphaBuffer[10] = std::min(GET(GET(GET(Rules, 10), 0), 0)(x_0), GET(GET(GET(Rules, 10), 0), 1)(x_1));
        AlphaBuffer[11] = std::min(GET(GET(GET(Rules, 11), 0), 0)(x_0), GET(GET(GET(Rules, 11), 0), 1)(x_1));
        AlphaBuffer[12] = std::min(GET(GET(GET(Rules, 12), 0), 0)(x_0), GET(GET(GET(Rules, 12), 0), 1)(x_1));
        AlphaBuffer[13] = std::min(GET(GET(GET(Rules, 13), 0), 0)(x_0), GET(GET(GET(Rules, 13), 0), 1)(x_1));
        AlphaBuffer[14] = std::min(GET(GET(GET(Rules, 14), 0), 0)(x_0), GET(GET(GET(Rules, 14), 0), 1)(x_1));
        AlphaBuffer[15] = std::min(GET(GET(GET(Rules, 15), 0), 0)(x_0), GET(GET(GET(Rules, 15), 0), 1)(x_1));
        AlphaBuffer[16] = std::min(GET(GET(GET(Rules, 16), 0), 0)(x_0), GET(GET(GET(Rules, 16), 0), 1)(x_1));
        AlphaBuffer[17] = std::min(GET(GET(GET(Rules, 17), 0), 0)(x_0), GET(GET(GET(Rules, 17), 0), 1)(x_1));
        AlphaBuffer[18] = std::min(GET(GET(GET(Rules, 18), 0), 0)(x_0), GET(GET(GET(Rules, 18), 0), 1)(x_1));
        AlphaBuffer[19] = std::min(GET(GET(GET(Rules, 19), 0), 0)(x_0), GET(GET(GET(Rules, 19), 0), 1)(x_1));
        AlphaBuffer[20] = std::min(GET(GET(GET(Rules, 20), 0), 0)(x_0), GET(GET(GET(Rules, 20), 0), 1)(x_1));
        AlphaBuffer[21] = std::min(GET(GET(GET(Rules, 21), 0), 0)(x_0), GET(GET(GET(Rules, 21), 0), 1)(x_1));
        AlphaBuffer[22] = std::min(GET(GET(GET(Rules, 22), 0), 0)(x_0), GET(GET(GET(Rules, 22), 0), 1)(x_1));
        AlphaBuffer[23] = std::min(GET(GET(GET(Rules, 23), 0), 0)(x_0), GET(GET(GET(Rules, 23), 0), 1)(x_1));
        AlphaBuffer[24] = std::min(GET(GET(GET(Rules, 24), 0), 0)(x_0), GET(GET(GET(Rules, 24), 0), 1)(x_1));
        AlphaBuffer[25] = std::min(GET(GET(GET(Rules, 25), 0), 0)(x_0), GET(GET(GET(Rules, 25), 0), 1)(x_1));
        AlphaBuffer[26] = std::min(GET(GET(GET(Rules, 26), 0), 0)(x_0), GET(GET(GET(Rules, 26), 0), 1)(x_1));
        AlphaBuffer[27] = std::min(GET(GET(GET(Rules, 27), 0), 0)(x_0), GET(GET(GET(Rules, 27), 0), 1)(x_1));
        AlphaBuffer[28] = std::min(GET(GET(GET(Rules, 28), 0), 0)(x_0), GET(GET(GET(Rules, 28), 0), 1)(x_1));
        AlphaBuffer[29] = std::min(GET(GET(GET(Rules, 29), 0), 0)(x_0), GET(GET(GET(Rules, 29), 0), 1)(x_1));
        AlphaBuffer[30] = std::min(GET(GET(GET(Rules, 30), 0), 0)(x_0), GET(GET(GET(Rules, 30), 0), 1)(x_1));
        AlphaBuffer[31] = std::min(GET(GET(GET(Rules, 31), 0), 0)(x_0), GET(GET(GET(Rules, 31), 0), 1)(x_1));
        AlphaBuffer[32] = std::min(GET(GET(GET(Rules, 32), 0), 0)(x_0), GET(GET(GET(Rules, 32), 0), 1)(x_1));
        AlphaBuffer[33] = std::min(GET(GET(GET(Rules, 33), 0), 0)(x_0), GET(GET(GET(Rules, 33), 0), 1)(x_1));
        AlphaBuffer[34] = std::min(GET(GET(GET(Rules, 34), 0), 0)(x_0), GET(GET(GET(Rules, 34), 0), 1)(x_1));
        AlphaBuffer[35] = std::min(GET(GET(GET(Rules, 35), 0), 0)(x_0), GET(GET(GET(Rules, 35), 0), 1)(x_1));
        AlphaBuffer[36] = std::min(GET(GET(GET(Rules, 36), 0), 0)(x_0), GET(GET(GET(Rules, 36), 0), 1)(x_1));
        AlphaBuffer[37] = std::min(GET(GET(GET(Rules, 37), 0), 0)(x_0), GET(GET(GET(Rules, 37), 0), 1)(x_1));
        AlphaBuffer[38] = std::min(GET(GET(GET(Rules, 38), 0), 0)(x_0), GET(GET(GET(Rules, 38), 0), 1)(x_1));
        AlphaBuffer[39] = std::min(GET(GET(GET(Rules, 39), 0), 0)(x_0), GET(GET(GET(Rules, 39), 0), 1)(x_1));
        AlphaBuffer[40] = std::min(GET(GET(GET(Rules, 40), 0), 0)(x_0), GET(GET(GET(Rules, 40), 0), 1)(x_1));
        AlphaBuffer[41] = std::min(GET(GET(GET(Rules, 41), 0), 0)(x_0), GET(GET(GET(Rules, 41), 0), 1)(x_1));
        AlphaBuffer[42] = std::min(GET(GET(GET(Rules, 42), 0), 0)(x_0), GET(GET(GET(Rules, 42), 0), 1)(x_1));
        AlphaBuffer[43] = std::min(GET(GET(GET(Rules, 43), 0), 0)(x_0), GET(GET(GET(Rules, 43), 0), 1)(x_1));
        AlphaBuffer[44] = std::min(GET(GET(GET(Rules, 44), 0), 0)(x_0), GET(GET(GET(Rules, 44), 0), 1)(x_1));
        AlphaBuffer[45] = std::min(GET(GET(GET(Rules, 45), 0), 0)(x_0), GET(GET(GET(Rules, 45), 0), 1)(x_1));
        AlphaBuffer[46] = std::min(GET(GET(GET(Rules, 46), 0), 0)(x_0), GET(GET(GET(Rules, 46), 0), 1)(x_1));
        AlphaBuffer[47] = std::min(GET(GET(GET(Rules, 47), 0), 0)(x_0), GET(GET(GET(Rules, 47), 0), 1)(x_1));
        AlphaBuffer[48] = std::min(GET(GET(GET(Rules, 48), 0), 0)(x_0), GET(GET(GET(Rules, 48), 0), 1)(x_1));
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
            MaxBuffer[7] = std::min(AlphaBuffer[7], GET(GET(GET(Rules, 7), 1), 0)(z_a + h * i));
            MaxBuffer[8] = std::min(AlphaBuffer[8], GET(GET(GET(Rules, 8), 1), 0)(z_a + h * i));
            MaxBuffer[9] = std::min(AlphaBuffer[9], GET(GET(GET(Rules, 9), 1), 0)(z_a + h * i));
            MaxBuffer[10] = std::min(AlphaBuffer[10], GET(GET(GET(Rules, 10), 1), 0)(z_a + h * i));
            MaxBuffer[11] = std::min(AlphaBuffer[11], GET(GET(GET(Rules, 11), 1), 0)(z_a + h * i));
            MaxBuffer[12] = std::min(AlphaBuffer[12], GET(GET(GET(Rules, 12), 1), 0)(z_a + h * i));
            MaxBuffer[13] = std::min(AlphaBuffer[13], GET(GET(GET(Rules, 13), 1), 0)(z_a + h * i));
            MaxBuffer[14] = std::min(AlphaBuffer[14], GET(GET(GET(Rules, 14), 1), 0)(z_a + h * i));
            MaxBuffer[15] = std::min(AlphaBuffer[15], GET(GET(GET(Rules, 15), 1), 0)(z_a + h * i));
            MaxBuffer[16] = std::min(AlphaBuffer[16], GET(GET(GET(Rules, 16), 1), 0)(z_a + h * i));
            MaxBuffer[17] = std::min(AlphaBuffer[17], GET(GET(GET(Rules, 17), 1), 0)(z_a + h * i));
            MaxBuffer[18] = std::min(AlphaBuffer[18], GET(GET(GET(Rules, 18), 1), 0)(z_a + h * i));
            MaxBuffer[19] = std::min(AlphaBuffer[19], GET(GET(GET(Rules, 19), 1), 0)(z_a + h * i));
            MaxBuffer[20] = std::min(AlphaBuffer[20], GET(GET(GET(Rules, 20), 1), 0)(z_a + h * i));
            MaxBuffer[21] = std::min(AlphaBuffer[21], GET(GET(GET(Rules, 21), 1), 0)(z_a + h * i));
            MaxBuffer[22] = std::min(AlphaBuffer[22], GET(GET(GET(Rules, 22), 1), 0)(z_a + h * i));
            MaxBuffer[23] = std::min(AlphaBuffer[23], GET(GET(GET(Rules, 23), 1), 0)(z_a + h * i));
            MaxBuffer[24] = std::min(AlphaBuffer[24], GET(GET(GET(Rules, 24), 1), 0)(z_a + h * i));
            MaxBuffer[25] = std::min(AlphaBuffer[25], GET(GET(GET(Rules, 25), 1), 0)(z_a + h * i));
            MaxBuffer[26] = std::min(AlphaBuffer[26], GET(GET(GET(Rules, 26), 1), 0)(z_a + h * i));
            MaxBuffer[27] = std::min(AlphaBuffer[27], GET(GET(GET(Rules, 27), 1), 0)(z_a + h * i));
            MaxBuffer[28] = std::min(AlphaBuffer[28], GET(GET(GET(Rules, 28), 1), 0)(z_a + h * i));
            MaxBuffer[29] = std::min(AlphaBuffer[29], GET(GET(GET(Rules, 29), 1), 0)(z_a + h * i));
            MaxBuffer[30] = std::min(AlphaBuffer[30], GET(GET(GET(Rules, 30), 1), 0)(z_a + h * i));
            MaxBuffer[31] = std::min(AlphaBuffer[31], GET(GET(GET(Rules, 31), 1), 0)(z_a + h * i));
            MaxBuffer[32] = std::min(AlphaBuffer[32], GET(GET(GET(Rules, 32), 1), 0)(z_a + h * i));
            MaxBuffer[33] = std::min(AlphaBuffer[33], GET(GET(GET(Rules, 33), 1), 0)(z_a + h * i));
            MaxBuffer[34] = std::min(AlphaBuffer[34], GET(GET(GET(Rules, 34), 1), 0)(z_a + h * i));
            MaxBuffer[35] = std::min(AlphaBuffer[35], GET(GET(GET(Rules, 35), 1), 0)(z_a + h * i));
            MaxBuffer[36] = std::min(AlphaBuffer[36], GET(GET(GET(Rules, 36), 1), 0)(z_a + h * i));
            MaxBuffer[37] = std::min(AlphaBuffer[37], GET(GET(GET(Rules, 37), 1), 0)(z_a + h * i));
            MaxBuffer[38] = std::min(AlphaBuffer[38], GET(GET(GET(Rules, 38), 1), 0)(z_a + h * i));
            MaxBuffer[39] = std::min(AlphaBuffer[39], GET(GET(GET(Rules, 39), 1), 0)(z_a + h * i));
            MaxBuffer[40] = std::min(AlphaBuffer[40], GET(GET(GET(Rules, 40), 1), 0)(z_a + h * i));
            MaxBuffer[41] = std::min(AlphaBuffer[41], GET(GET(GET(Rules, 41), 1), 0)(z_a + h * i));
            MaxBuffer[42] = std::min(AlphaBuffer[42], GET(GET(GET(Rules, 42), 1), 0)(z_a + h * i));
            MaxBuffer[43] = std::min(AlphaBuffer[43], GET(GET(GET(Rules, 43), 1), 0)(z_a + h * i));
            MaxBuffer[44] = std::min(AlphaBuffer[44], GET(GET(GET(Rules, 44), 1), 0)(z_a + h * i));
            MaxBuffer[45] = std::min(AlphaBuffer[45], GET(GET(GET(Rules, 45), 1), 0)(z_a + h * i));
            MaxBuffer[46] = std::min(AlphaBuffer[46], GET(GET(GET(Rules, 46), 1), 0)(z_a + h * i));
            MaxBuffer[47] = std::min(AlphaBuffer[47], GET(GET(GET(Rules, 47), 1), 0)(z_a + h * i));
            MaxBuffer[48] = std::min(AlphaBuffer[48], GET(GET(GET(Rules, 48), 1), 0)(z_a + h * i));
#undef GET
            double ro_dV =  h*(*(std::max_element(MaxBuffer.begin(), MaxBuffer.end())));
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
    double Get_a_x() 
    {
        return x_a;
    }
    double Get_b_x()
    {
        return x_b;
    }
    double Get_a_y()
    {
        return y_a;
    }
    double Get_b_y()
    {
        return y_b;
    }
    double Get_a_z()
    {
        return z_a;
    }
    double Get_b_z()
    {
        return z_b;
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


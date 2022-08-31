#pragma once

#include "FuzzyInference.h"

/// CONTROLLER CONSTRUCTION START

//NB;   -10.0,-1.0,-2/3
//NM;   -1.0,-2/3,-1/3
//NS;   -2/3,-1/3,0.0
//ZE;   -1/3,0,1/3
//PS;   0,1/3,2/3
//PM;   1/3,2/3,1.0
//PB;   2/3,1.0,10.0

namespace CR_API {
    double a_x = -1.0;
    double b_x = 1.0;
    double a_y = -1.0;
    double b_y = 1.0;
    double a_z = -1.0;
    double b_z = 1.0;

    // value that reterns when numerator in fuzzy inferense is zero
    double signal_value = -999.0;


    const auto theta_NB = TriangleMembershipFunction(-10.0, -1.0, -2 / 3);
    const auto theta_NM = TriangleMembershipFunction(-1.0, -2 / 3, -1 / 3);
    const auto theta_NS = TriangleMembershipFunction(-2 / 3, -1 / 3, 0.0);
    const auto theta_ZE = TriangleMembershipFunction(-1 / 3, 0, 1 / 3);
    const auto theta_PS = TriangleMembershipFunction(0, 1 / 3, 2 / 3);
    const auto theta_PM = TriangleMembershipFunction(1 / 3, 2 / 3, 1.0);
    const auto theta_PB = TriangleMembershipFunction(2 / 3, 1.0, 10.0);

    const auto omega_NB = TriangleMembershipFunction(-10.0, -1.0, -2 / 3);
    const auto omega_NM = TriangleMembershipFunction(-1.0, -2 / 3, -1 / 3);
    const auto omega_NS = TriangleMembershipFunction(-2 / 3, -1 / 3, 0.0);
    const auto omega_ZE = TriangleMembershipFunction(-1 / 3, 0, 1 / 3);
    const auto omega_PS = TriangleMembershipFunction(0, 1 / 3, 2 / 3);
    const auto omega_PM = TriangleMembershipFunction(1 / 3, 2 / 3, 1.0);
    const auto omega_PB = TriangleMembershipFunction(2 / 3, 1.0, 10.0);

    const auto f_NB = TriangleMembershipFunction(-10.0, -1.0, -2 / 3);
    const auto f_NM = TriangleMembershipFunction(-1.0, -2 / 3, -1 / 3);
    const auto f_NS = TriangleMembershipFunction(-2 / 3, -1 / 3, 0.0);
    const auto f_ZE = TriangleMembershipFunction(-1 / 3, 0, 1 / 3);
    const auto f_PS = TriangleMembershipFunction(0, 1 / 3, 2 / 3);
    const auto f_PM = TriangleMembershipFunction(1 / 3, 2 / 3, 1.0);
    const auto f_PB = TriangleMembershipFunction(2 / 3, 1.0, 10.0);

    // theta is PB
    auto RULE_0 = RULE(IF(theta_PB, omega_NB), THEN(f_ZE));
    auto RULE_1 = RULE(IF(theta_PB, omega_NM), THEN(f_PS));
    auto RULE_2 = RULE(IF(theta_PB, omega_NS), THEN(f_PM));
    auto RULE_3 = RULE(IF(theta_PB, omega_ZE), THEN(f_PB));
    auto RULE_4 = RULE(IF(theta_PB, omega_PS), THEN(f_PB));
    auto RULE_5 = RULE(IF(theta_PB, omega_PM), THEN(f_PB));
    auto RULE_6 = RULE(IF(theta_PB, omega_PB), THEN(f_PB));

    // theta is PM
    auto RULE_7 = RULE(IF(theta_PM, omega_NB), THEN(f_NS));
    auto RULE_8 = RULE(IF(theta_PM, omega_NM), THEN(f_ZE));
    auto RULE_9 = RULE(IF(theta_PM, omega_NS), THEN(f_PS));
    auto RULE_10 = RULE(IF(theta_PM, omega_ZE), THEN(f_PM));
    auto RULE_11 = RULE(IF(theta_PM, omega_PS), THEN(f_PB));
    auto RULE_12 = RULE(IF(theta_PM, omega_PM), THEN(f_PB));
    auto RULE_13 = RULE(IF(theta_PM, omega_PB), THEN(f_PB));

    // theta is PS
    auto RULE_14 = RULE(IF(theta_PS, omega_NB), THEN(f_NM));
    auto RULE_15 = RULE(IF(theta_PS, omega_NM), THEN(f_NS));
    auto RULE_16 = RULE(IF(theta_PS, omega_NS), THEN(f_ZE));
    auto RULE_17 = RULE(IF(theta_PS, omega_ZE), THEN(f_PS));
    auto RULE_18 = RULE(IF(theta_PS, omega_PS), THEN(f_PM));
    auto RULE_19 = RULE(IF(theta_PS, omega_PM), THEN(f_PB));
    auto RULE_20 = RULE(IF(theta_PS, omega_PB), THEN(f_PB));

    // theta is ZE
    auto RULE_21 = RULE(IF(theta_ZE, omega_NB), THEN(f_NB));
    auto RULE_22 = RULE(IF(theta_ZE, omega_NM), THEN(f_NM));
    auto RULE_23 = RULE(IF(theta_ZE, omega_NS), THEN(f_NS));
    auto RULE_24 = RULE(IF(theta_ZE, omega_ZE), THEN(f_ZE));
    auto RULE_25 = RULE(IF(theta_ZE, omega_PS), THEN(f_PS));
    auto RULE_26 = RULE(IF(theta_ZE, omega_PM), THEN(f_PM));
    auto RULE_27 = RULE(IF(theta_ZE, omega_PB), THEN(f_PB));

    // theta is NS
    auto RULE_28 = RULE(IF(theta_NS, omega_NB), THEN(f_NB));
    auto RULE_29 = RULE(IF(theta_NS, omega_NM), THEN(f_NB));
    auto RULE_30 = RULE(IF(theta_NS, omega_NS), THEN(f_NM));
    auto RULE_31 = RULE(IF(theta_NS, omega_ZE), THEN(f_NS));
    auto RULE_32 = RULE(IF(theta_NS, omega_PS), THEN(f_ZE));
    auto RULE_33 = RULE(IF(theta_NS, omega_PM), THEN(f_PS));
    auto RULE_34 = RULE(IF(theta_NS, omega_PB), THEN(f_PM));

    // theta is NM
    auto RULE_35 = RULE(IF(theta_NM, omega_NB), THEN(f_NB));
    auto RULE_36 = RULE(IF(theta_NM, omega_NM), THEN(f_NB));
    auto RULE_37 = RULE(IF(theta_NM, omega_NS), THEN(f_NB));
    auto RULE_38 = RULE(IF(theta_NM, omega_ZE), THEN(f_NM));
    auto RULE_39 = RULE(IF(theta_NM, omega_PS), THEN(f_NS));
    auto RULE_40 = RULE(IF(theta_NM, omega_PM), THEN(f_ZE));
    auto RULE_41 = RULE(IF(theta_NM, omega_PB), THEN(f_PS));

    // theta is NB
    auto RULE_42 = RULE(IF(theta_NB, omega_NB), THEN(f_NB));
    auto RULE_43 = RULE(IF(theta_NB, omega_NM), THEN(f_NB));
    auto RULE_44 = RULE(IF(theta_NB, omega_NS), THEN(f_NB));
    auto RULE_45 = RULE(IF(theta_NB, omega_ZE), THEN(f_NB));
    auto RULE_46 = RULE(IF(theta_NB, omega_PS), THEN(f_NM));
    auto RULE_47 = RULE(IF(theta_NB, omega_PM), THEN(f_NS));
    auto RULE_48 = RULE(IF(theta_NB, omega_PB), THEN(f_ZE));


    auto Controller = MAKE_FUZZY_CONTROLLER(a_x, b_x, a_y, b_y, a_z, b_z, signal_value, 100, 49,
        RULE_0, RULE_1, RULE_2, RULE_3, RULE_4, RULE_5, RULE_6, RULE_7, RULE_8, RULE_9, RULE_10, RULE_11, RULE_12, RULE_13, RULE_14, RULE_15, RULE_16, RULE_17, RULE_18, RULE_19, RULE_20, RULE_21, RULE_22, RULE_23, RULE_24, RULE_25, RULE_26, RULE_27, RULE_28, RULE_29, RULE_30, RULE_31, RULE_32, RULE_33, RULE_34, RULE_35, RULE_36, RULE_37, RULE_38, RULE_39, RULE_40, RULE_41, RULE_42, RULE_43, RULE_44, RULE_45, RULE_46, RULE_47, RULE_48);

    /// CONTROLLER CONSTRUCTION END

    // hide the variables
    using ControllerType = decltype(Controller);

    ControllerType GetController()
    {
        return ControllerType(Controller);
    }
}

#undef IF
#undef THEN
#undef RULE
#undef MAKE_RULES
#undef MAKE_FUZZY_CONTROLLER


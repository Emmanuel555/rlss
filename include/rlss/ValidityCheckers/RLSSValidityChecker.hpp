#ifndef RLSS_RLSS_VALIDITY_CHECKER_HPP
#define RLSS_RLSS_VALIDITY_CHECKER_HPP

#include <rlss/internal/Util.hpp>
#include <rlss/ValidityCheckers/ValidityChecker.hpp>

namespace rlss {
template<typename T, unsigned int DIM>
class RLSSValidityChecker: public ValidityChecker<T, DIM> {
public:
    using Base = ValidityChecker<T, DIM>;
    using PiecewiseCurve = typename Base::PiecewiseCurve;

    RLSSValidityChecker(
        const std::vector<std::pair<unsigned int, T>>& maxdermag,
        T search_step
    ): m_max_derivative_magnitudes(maxdermag),
       m_search_step(search_step)
    {

    }

    bool isValid(const PiecewiseCurve &curve) override {
    
        for(const auto& [d, l]: m_max_derivative_magnitudes) {
            //std::cout << "need to do stupid validity check 1..." << d << std::endl;
            //std::cout << "need to do stupid validity check 2..." << l << std::endl;
            for(
                T param = 0;
                param < curve.maxParameter();
                param += m_search_step
            ) {
                T norm = curve.eval(param, d).norm(); //curve.eval(time_param, derivative)

                if (d == 1)
                {
                    std::cout << "Currently traversing at velocity " << norm << std::endl;
                }
                //std::cout << "time maxparam of curve..." << curve.maxParameter() << std::endl;
                //std::cout << "euclidean distance from goal to origin at 1st derivative..." << (curve.eval(curve.maxParameter(),d)).norm() << std::endl;
                //std::cout << "euclidean distance from goal to origin at position..." << (curve.eval(curve.maxParameter(),0)).norm() << std::endl;
                /*std::cout << "Params..." << param << std::endl;
                std::cout << "d value..." << d << std::endl;
                std::cout << "Norm..." << norm << std::endl;
                std::cout << "l.." << l << std::endl;*/

                if(norm > l) {
                    /*std::cout << "time maxparam of curve..." << curve.maxParameter() << std::endl;
                    std::cout << "error_Params..." << param << std::endl;
                    std::cout << "error_d value..." << d << std::endl;
                    std::cout << "error_Norm..." << norm << std::endl;
                    std::cout << "error_l.." << l << std::endl;*/
                    debug_message(
                            internal::debug::colors::RED,
                            "norm of the ",
                            d,
                            "th degree of curve's derivative at ",
                            param,
                            " is ",
                            norm,
                            " while the maximum allowed is ",
                            l,
                            internal::debug::colors::RESET
                    );
                    return false;
                }
            }
        }
        return true;
    }


private:
    std::vector<std::pair<unsigned int, T>> m_max_derivative_magnitudes;
    T m_search_step;
};
} // namespace rlss
#endif // RLSS_RLSS_VALIDITY_CHECKER_HPP
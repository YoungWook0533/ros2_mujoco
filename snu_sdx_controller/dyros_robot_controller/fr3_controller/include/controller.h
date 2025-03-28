#ifndef FR3_CONTROLLER_HPP
#define FR3_CONTROLLER_HPP

#include <Eigen/Dense>

using namespace Eigen;

namespace FR3Controller
{
    class Controller
    {
    public:
        Controller(double dt);
        ~Controller();

        VectorXd tmpControl();

    private :
        double dt_;

    };
} // namespace Controller

#endif // FR3_CONTROLLER_HPP
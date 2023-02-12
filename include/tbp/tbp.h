#include <array>
#include <boost/qvm/vec.hpp>

namespace fr_tbp {

class ThreeBodyProblem {
  public:
    ThreeBodyProblem(std::array<boost::qvm::vec<double, 3>, 3> bodyPositions,
                     std::array<double, 3> masses, double deltaTime)
        : _bodyPositions(bodyPositions), _masses(masses), _deltaTime(deltaTime){};

    void simulate();

  private:
    std::array<boost::qvm::vec<double, 3>, 3> _bodyPositions;
    std::array<boost::qvm::vec<double, 3>, 3> _bodyVelocities{};
    std::array<double, 3> _masses;
    double _deltaTime;

    std::array<boost::qvm::vec<double, 3>, 3> _calculateBodyAccelerations();
    void _updateBodyPositions(std::array<boost::qvm::vec<double, 3>, 3> accelerations);
    void _updateVelocities(std::array<boost::qvm::vec<double, 3>, 3> accelerations);
};

} // namespace fr_tbp

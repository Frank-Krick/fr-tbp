#include <array>
#include <boost/qvm/vec.hpp>

namespace fr::tbp {

class ThreeBodyProblem {
  public:
    ThreeBodyProblem(std::array<boost::qvm::vec<double, 3>, 3> bodyPositions,
                     std::array<boost::qvm::vec<double, 3>, 3> bodyVelocities,
                     std::array<double, 3> masses, double deltaTime,
                     double gravitationalConstant)
        : _bodyPositions(bodyPositions), _masses(masses), _deltaTime(deltaTime),
          _gravitationalConstant(gravitationalConstant),
          _bodyVelocities(bodyVelocities){};

    void simulate();

    std::array<boost::qvm::vec<double, 3>, 3> bodyPositions() {
        return _bodyPositions;
    }

    void setGravitationalConstant(double gravitationalConstant) {
        _gravitationalConstant = gravitationalConstant;
    }

    [[nodiscard]] double gravitationalConstant() const {
        return _gravitationalConstant;
    }

  private:
    std::array<boost::qvm::vec<double, 3>, 3> _bodyPositions;
    std::array<boost::qvm::vec<double, 3>, 3> _bodyVelocities;
    std::array<double, 3> _masses;
    double _deltaTime;
    double _gravitationalConstant;

    std::array<boost::qvm::vec<double, 3>, 3> _calculateBodyAccelerations();
    void _updateBodyPositions();
    void
    _updateVelocities(std::array<boost::qvm::vec<double, 3>, 3> accelerations);
};

} // namespace fr::tbp

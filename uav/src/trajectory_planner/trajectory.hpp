/**
 * trajectory.hpp
 *
 * Piecewise polynomial trajectory representation for UAV motion planning.
 *
 * Each piece is a polynomial of degree D (default 7 for minimum-snap).
 * Coefficients are stored in a 3x(D+1) matrix where row 0=x, 1=y, 2=z,
 * and columns correspond to powers of t: col j = coefficient of t^j.
 *
 * So position p(t) = coeff * [1, t, t^2, ..., t^D]^T
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 *
 * References:
 *   Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for
 *   Quadrotors", ICRA 2011.
 */

#ifndef UAV_TRAJECTORY_HPP
#define UAV_TRAJECTORY_HPP

#include <vector>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

namespace uav_planning {

/**
 * @brief A single polynomial piece of degree D.
 *
 * Stores a duration T and a 3x(D+1) coefficient matrix.
 * The polynomial is defined on [0, T].
 * Row i (i=0,1,2) corresponds to x, y, z axis.
 * Column j corresponds to the coefficient of t^j.
 *
 * p_i(t) = sum_{j=0}^{D} coeff(i,j) * t^j,   t in [0, T]
 *
 * @tparam D Polynomial degree (default 7 for minimum-snap).
 */
template <int D = 7>
class Piece
{
public:
    // Coefficient matrix: 3 rows (x,y,z), D+1 cols
    typedef Eigen::Matrix<double, 3, D + 1> CoeffMatrix;

    Piece() : duration_(1.0), coeffs_(CoeffMatrix::Zero()) {}

    Piece(double duration, const CoeffMatrix& coeffs)
        : duration_(duration), coeffs_(coeffs)
    {
        assert(duration > 0.0);
    }

    double getDuration() const { return duration_; }

    const CoeffMatrix& getCoeffs() const { return coeffs_; }
    CoeffMatrix& getCoeffs() { return coeffs_; }

    void setDuration(double d)
    {
        assert(d > 0.0);
        duration_ = d;
    }

    void setCoeffs(const CoeffMatrix& c) { coeffs_ = c; }

    // -------------------------------------------------------------------------
    // Evaluate position at local time t (t must be in [0, duration_])
    // -------------------------------------------------------------------------
    Eigen::Vector3d getPos(double t) const
    {
        return evalPoly(t, 0);
    }

    // -------------------------------------------------------------------------
    // Evaluate velocity (1st derivative) at local time t
    // -------------------------------------------------------------------------
    Eigen::Vector3d getVel(double t) const
    {
        return evalPoly(t, 1);
    }

    // -------------------------------------------------------------------------
    // Evaluate acceleration (2nd derivative) at local time t
    // -------------------------------------------------------------------------
    Eigen::Vector3d getAcc(double t) const
    {
        return evalPoly(t, 2);
    }

    // -------------------------------------------------------------------------
    // Evaluate jerk (3rd derivative) at local time t
    // -------------------------------------------------------------------------
    Eigen::Vector3d getJer(double t) const
    {
        return evalPoly(t, 3);
    }

    // -------------------------------------------------------------------------
    // Evaluate snap (4th derivative) at local time t
    // -------------------------------------------------------------------------
    Eigen::Vector3d getSnap(double t) const
    {
        return evalPoly(t, 4);
    }

private:
    double duration_;
    CoeffMatrix coeffs_;

    /**
     * Evaluate the n-th derivative of the polynomial at time t.
     * Uses Horner's method for numerical stability and efficiency.
     *
     * The n-th derivative of sum_{j=0}^{D} c_j t^j is:
     *   sum_{j=n}^{D} c_j * (j! / (j-n)!) * t^{j-n}
     */
    Eigen::Vector3d evalPoly(double t, int deriv) const
    {
        Eigen::Vector3d result = Eigen::Vector3d::Zero();

        // Compute factorial-like coefficients for derivative
        // For derivative order 'deriv', coeff of t^{j-deriv} is c_j * prod(k, j-deriv+1, j)
        // We accumulate using Horner's scheme starting from the highest power.

        // Number of terms in the derivative polynomial
        int terms = D + 1 - deriv;
        if (terms <= 0) return result;

        // Build derivative coefficient vector for each axis
        // d_j = c_{j+deriv} * product_{k=0}^{deriv-1}(j + deriv - k)
        //     = c_{j+deriv} * (j+deriv)! / j!
        // for j = 0 .. D-deriv

        // Use Horner's method: start from highest index
        // result = d_{terms-1}
        // result = result * t + d_{terms-2}
        // ...
        // result = result * t + d_0

        for (int axis = 0; axis < 3; axis++)
        {
            double val = 0.0;
            // Horner from high to low (power D-deriv down to 0)
            for (int j = D - deriv; j >= 0; j--)
            {
                // Derivative coefficient: c_{j+deriv} * prod_{k=0}^{deriv-1}(j+deriv-k)
                double dc = coeffs_(axis, j + deriv);
                double factor = 1.0;
                for (int k = 0; k < deriv; k++)
                {
                    factor *= static_cast<double>(j + deriv - k);
                }
                val = val * t + dc * factor;
            }
            result(axis) = val;
        }

        return result;
    }
};

// =============================================================================

/**
 * @brief Piecewise polynomial trajectory composed of N pieces of degree D.
 *
 * Global time t maps to a specific piece via locatePiece().
 *
 * @tparam D Polynomial degree per piece (default 7 for minimum-snap).
 */
template <int D = 7>
class Trajectory
{
public:
    typedef Piece<D> PieceType;

    Trajectory() {}

    explicit Trajectory(const std::vector<PieceType>& pieces)
        : pieces_(pieces) {}

    // -------------------------------------------------------------------------
    void clear() { pieces_.clear(); }

    void addPiece(const PieceType& p) { pieces_.push_back(p); }

    void addPiece(double duration, const typename PieceType::CoeffMatrix& coeffs)
    {
        pieces_.push_back(PieceType(duration, coeffs));
    }

    int getPieceNum() const { return static_cast<int>(pieces_.size()); }

    const PieceType& getPiece(int i) const { return pieces_[i]; }
    PieceType& getPiece(int i) { return pieces_[i]; }

    // -------------------------------------------------------------------------
    double getTotalDuration() const
    {
        double total = 0.0;
        for (int i = 0; i < static_cast<int>(pieces_.size()); i++)
            total += pieces_[i].getDuration();
        return total;
    }

    // -------------------------------------------------------------------------
    /**
     * @brief Locate which piece corresponds to global time t.
     *
     * Modifies t so that on return, t is the LOCAL time within the found piece.
     * Returns the piece index. Clamps to valid range.
     *
     * @param t [in/out] Global time -> local time within piece
     * @return   Piece index in [0, N-1]
     */
    int locatePiece(double& t) const
    {
        assert(!pieces_.empty());

        // Clamp to [0, totalDuration]
        if (t <= 0.0)
        {
            t = 0.0;
            return 0;
        }

        int n = static_cast<int>(pieces_.size());
        for (int i = 0; i < n - 1; i++)
        {
            double d = pieces_[i].getDuration();
            if (t <= d)
                return i;
            t -= d;
        }

        // Last piece — clamp t to its duration
        int last = n - 1;
        double lastDur = pieces_[last].getDuration();
        if (t > lastDur)
            t = lastDur;
        return last;
    }

    // -------------------------------------------------------------------------
    // Evaluate the trajectory at global time t
    // -------------------------------------------------------------------------

    Eigen::Vector3d getPos(double t) const
    {
        int idx = locatePiece(t);
        return pieces_[idx].getPos(t);
    }

    Eigen::Vector3d getVel(double t) const
    {
        int idx = locatePiece(t);
        return pieces_[idx].getVel(t);
    }

    Eigen::Vector3d getAcc(double t) const
    {
        int idx = locatePiece(t);
        return pieces_[idx].getAcc(t);
    }

    Eigen::Vector3d getJer(double t) const
    {
        int idx = locatePiece(t);
        return pieces_[idx].getJer(t);
    }

    Eigen::Vector3d getSnap(double t) const
    {
        int idx = locatePiece(t);
        return pieces_[idx].getSnap(t);
    }

    // -------------------------------------------------------------------------
    bool empty() const { return pieces_.empty(); }

    const std::vector<PieceType>& getPieces() const { return pieces_; }

private:
    std::vector<PieceType> pieces_;
};

} // namespace uav_planning

#endif // UAV_TRAJECTORY_HPP

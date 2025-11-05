#include <iostream>

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <vector>

// ------------------------------------------------------------------
// Compute arbitrary N-anchor coordinates from symmetric distance matrix
// - Fix anchor0 at (0,0,0)
// - Place anchor1 on X-axis
// - Place anchor2 in XY plane
// - Solve remaining anchors via least squares sphere intersection
// ------------------------------------------------------------------

Eigen::MatrixXd computeAnchorLayout(const Eigen::MatrixXd& d) {
    const int N = d.rows();
    if (d.cols() != N) throw std::invalid_argument("Distance matrix must be square.");
    if (N < 3) throw std::invalid_argument("Need at least 3 anchors.");

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, 3);

    // Fix base anchors
    double a = d(0,1);
    double b = d(0,2);
    double c = d(1,2);

    // Triangle check
    if (a+b<=c || a+c<=b || b+c<=a)
        throw std::invalid_argument("Invalid base triangle distances.");

    // Anchor0 at origin
    A.row(0) << 0,0,0;

    // Anchor1 on x-axis
    A.row(1) << a,0,0;

    // Anchor2 in XY-plane
    double x2 = (a*a + b*b - c*c) / (2*a);
    double y2 = std::sqrt(std::max(0.0, b*b - x2*x2));
    A.row(2) << x2, y2, 0;

    // Compute remaining anchors
    for (int k = 3; k < N; ++k) {
        const int refCount = std::min(k, 4);  // use up to 4 references for stability
        Eigen::MatrixXd M(refCount, 3);
        Eigen::VectorXd r(refCount);

        for (int i = 0; i < refCount; ++i) {
            double xi=A(i,0), yi=A(i,1), zi=A(i,2);
            M.row(i) << 2*xi, 2*yi, 2*zi;
            r(i) = xi*xi + yi*yi + zi*zi - d(k,i)*d(k,i);
        }

        // Solve least squares for position
        Eigen::Vector3d p = (M.transpose()*M).ldlt().solve(M.transpose()*r);
        A.row(k) = p.transpose();
    }

    return A;
}

int main(int argc, char* argv[]) {
	return 0;
}


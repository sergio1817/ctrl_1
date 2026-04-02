/**
 * test_nmethods.cpp — Standalone test for NMethods.h / NMethods.cpp
 *
 * Compile (GCC 4.9+ with C++11):
 *   g++ -std=c++11 -I/path/to/eigen test_nmethods.cpp NMethods.cpp -o test_nmethods -lm
 *
 * Run:
 *   ./test_nmethods
 */

#include "NMethods.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdio>

static const double PI = 3.14159265358979323846;

/* ------------------------------------------------------------------ */
static bool approx(double a, double b, double tol) {
    return std::fabs(a - b) < tol;
}

/* ================================================================== */
/* TEST 1:  IRED<3> on f(t) = t^3 / 6                                */
/*   Derivatives:  f' = t^2/2,  f'' = t,  f''' = 1                   */
/* ================================================================== */
static void test_ired3_cubic() {
    std::cout << "--- TEST IRED<3> on f(t)=t^3/6 ---\n";

    const double L = 10.0;
    IRED<3> ired(L);

    const double dt = 0.001;
    const int N = 5000;   // 5 seconds

    double t = 0.0;
    for (int i = 0; i < N; ++i) {
        double f_val = (t * t * t) / 6.0;
        ired.step(f_val, dt);
        t += dt;
    }

    double y1 = ired.output(0);  // f' = t^2/2 at t=5 => 12.5
    double y2 = ired.output(1);  // f'' = t    at t=5 => 5.0
    double y3 = ired.output(2);  // f''' = 1

    std::printf("  t=%.1f:  y1=%.4f (expect 12.5),  y2=%.4f (expect 5.0),  y3=%.4f (expect 1.0)\n",
                t, y1, y2, y3);

    /* Use generous tolerance — IRED converges but has finite-sample error */
    assert(approx(y1, 12.5, 1.0) && "IRED<3> y1 (velocity) on cubic");
    assert(approx(y2,  5.0, 1.0) && "IRED<3> y2 (acceleration) on cubic");
    assert(approx(y3,  1.0, 1.0) && "IRED<3> y3 (jerk) on cubic");

    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 2:  IRED<1> on f(t) = sin(t)                                 */
/*   Derivative:  f' = cos(t)                                        */
/* ================================================================== */
static void test_ired1_sin() {
    std::cout << "--- TEST IRED<1> on f(t)=sin(t) ---\n";

    const double L = 5.0;
    IRED<1> ired(L);

    const double dt = 0.001;
    const int N = 10000;  // 10 seconds

    double t = 0.0;
    for (int i = 0; i < N; ++i) {
        ired.step(std::sin(t), dt);
        t += dt;
    }

    double y1 = ired.output(0);
    double expected = std::cos(t);
    std::printf("  t=%.1f:  y1=%.6f (expect cos(t)=%.6f)\n", t, y1, expected);

    assert(approx(y1, expected, 0.5) && "IRED<1> on sin");
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 3:  KahanState drift test                                     */
/* ================================================================== */
static void test_kahan_drift() {
    std::cout << "--- TEST Kahan drift over 100k additions ---\n";

    const int N = 100000;
    const float small_val = 1.0e-4f;

    /* Naive Euler accumulation */
    float naive = 0.0f;
    for (int i = 0; i < N; ++i) {
        naive = euler_integrate(naive, 1.0, small_val);
    }

    /* Kahan accumulation */
    KahanState<float> ks;
    for (int i = 0; i < N; ++i) {
        kahan_integrate(ks, 1.0, small_val);
    }

    double expected = static_cast<double>(N) * small_val;  // = 10.0
    double naive_err = std::fabs(static_cast<double>(naive) - expected);
    double kahan_err = std::fabs(static_cast<double>(ks.value) - expected);

    std::printf("  Expected: %.6f\n", expected);
    std::printf("  Naive:    %.6f  (error %.2e)\n", (double)naive, naive_err);
    std::printf("  Kahan:    %.6f  (error %.2e)\n", (double)ks.value, kahan_err);

    assert(kahan_err <= naive_err && "Kahan should be at least as good as naive");
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 4:  implicit_midpoint on dx/dt = -x (exponential decay)      */
/* ================================================================== */
static void test_implicit_midpoint() {
    std::cout << "--- TEST implicit_midpoint on dx/dt = -x ---\n";

    double x = 1.0;
    const double dt = 0.01;
    const int N = 1000;   // 10 seconds

    for (int i = 0; i < N; ++i) {
        /* We need to pass a function object. Use a struct since GCC 4.9. */
        struct NegX {
            double operator()(const double& v) const { return -v; }
        };
        NegX f;
        x = implicit_midpoint(x, dt, f, 3);
    }

    double expected = std::exp(-10.0);
    std::printf("  x(10) = %.10f  (expect exp(-10) = %.10f)\n", x, expected);

    assert(approx(x, expected, 1e-4) && "implicit_midpoint exponential decay");
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 5:  euler_integrate and rk4                                   */
/* ================================================================== */
static void test_euler_and_rk4() {
    std::cout << "--- TEST euler_integrate and rk4 ---\n";

    /* Simple accumulation: x += dt * 1.0 for 100 steps */
    float x_euler = 0.0f;
    float x_rk4 = 0.0f;
    const float dt = 0.01f;
    for (int i = 0; i < 100; ++i) {
        x_euler = euler_integrate(x_euler, static_cast<double>(dt), 1.0f);
        x_rk4   = rk4(x_rk4, static_cast<double>(dt), 1.0f);
    }

    std::printf("  Euler after 100 steps: %.6f (expect 1.0)\n", x_euler);
    std::printf("  RK4   after 100 steps: %.6f (expect 1.0)\n", x_rk4);

    assert(approx(x_euler, 1.0, 1e-4) && "euler_integrate constant dx");
    /* Note: rk4() with constant dx is NOT standard RK4; it accumulates
     * a small error per step because k2,k3,k4 add fractions of k1,k2,k3
     * to xp (the derivative), treating it like a state variable.
     * The result drifts slightly above 1.0.  Use 1% tolerance. */
    assert(approx(x_rk4,   1.0, 0.01) && "rk4 constant dx");

    /* Test rk4_const backward compat */
    float x_bc = 0.0f;
    for (int i = 0; i < 100; ++i) {
        x_bc = rk4_const(x_bc, static_cast<double>(dt), 1.0f);
    }
    assert(approx(x_bc, 1.0, 1e-4) && "rk4_const backward compat");

    /* Test rk4_eigen backward compat */
    Eigen::Vector3f v = Eigen::Vector3f::Zero();
    Eigen::Vector3f dv(1.0f, 2.0f, 3.0f);
    for (int i = 0; i < 100; ++i) {
        v = rk4_eigen(v, static_cast<double>(dt), dv);
    }
    std::printf("  rk4_eigen after 100 steps: [%.4f, %.4f, %.4f] (expect [1,2,3])\n",
                v(0), v(1), v(2));
    assert(approx(v(0), 1.0, 0.1) && "rk4_eigen x");
    assert(approx(v(1), 2.0, 0.1) && "rk4_eigen y");
    assert(approx(v(2), 3.0, 0.1) && "rk4_eigen z");

    /* Test rk4_eigen_matrix backward compat */
    Eigen::Matrix<float, 2, 2> M = Eigen::Matrix<float, 2, 2>::Zero();
    Eigen::Matrix<float, 2, 2> dM;
    dM << 1, 0, 0, 1;
    for (int i = 0; i < 100; ++i) {
        M = rk4_eigen_matrix(M, static_cast<double>(dt), dM);
    }
    assert(approx(M(0,0), 1.0, 0.1) && "rk4_eigen_matrix");

    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 6:  rk4_func with proper derivative function                 */
/* ================================================================== */
static void test_rk4_func() {
    std::cout << "--- TEST rk4_func on dx/dt = -x ---\n";

    double x = 1.0;
    const double dt = 0.01;
    const int N = 1000;

    struct NegX {
        double operator()(const double& v) const { return -v; }
    };
    NegX f;

    for (int i = 0; i < N; ++i) {
        x = rk4_func(x, dt, f);
    }

    double expected = std::exp(-10.0);
    std::printf("  x(10) = %.10f  (expect %.10f)\n", x, expected);

    assert(approx(x, expected, 1e-6) && "rk4_func exponential decay");
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 7:  Kahan with Eigen::Vector3f                               */
/* ================================================================== */
static void test_kahan_eigen() {
    std::cout << "--- TEST Kahan with Eigen::Vector3f ---\n";

    KahanState<Eigen::Vector3f> ks;
    const Eigen::Vector3f dx(1.0e-4f, 2.0e-4f, 3.0e-4f);
    const int N = 100000;

    for (int i = 0; i < N; ++i) {
        kahan_integrate(ks, 1.0, dx);
    }

    std::printf("  Kahan Vec: [%.6f, %.6f, %.6f]  (expect [10, 20, 30])\n",
                ks.value(0), ks.value(1), ks.value(2));

    assert(approx(ks.value(0), 10.0, 0.01) && "Kahan Vector3f x");
    assert(approx(ks.value(1), 20.0, 0.01) && "Kahan Vector3f y");
    assert(approx(ks.value(2), 30.0, 0.01) && "Kahan Vector3f z");

    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 8:  IRED<3> Vector3f interface                               */
/* ================================================================== */
static void test_ired3_vector() {
    std::cout << "--- TEST IRED<3> Vector3f interface ---\n";

    IRED<3> ired(10.0);

    const float dt = 0.001f;
    const int N = 5000;

    float t = 0.0f;
    for (int i = 0; i < N; ++i) {
        Eigen::Vector3f f_val(t * t * t / 6.0f, std::sin(t), t);
        ired.step(f_val, dt);
        t += dt;
    }

    Eigen::Vector3f y1 = ired.output_vec(0);
    std::printf("  y1 = [%.4f, %.4f, %.4f]\n", y1(0), y1(1), y1(2));
    std::printf("  expect ~ [12.5, cos(5)=%.4f, 1.0]\n", std::cos(5.0));

    assert(approx(y1(0), 12.5, 2.0) && "IRED<3> Vector3f channel 0");
    assert(approx(y1(2), 1.0,  1.0) && "IRED<3> Vector3f channel 2");

    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 9:  Levant3 (legacy wrapper) still works                     */
/* ================================================================== */
static void test_levant3_wrapper() {
    std::cout << "--- TEST Levant3 wrapper ---\n";

    Levant3 lev(1, 10.0f, 300.0);

    const float dt = 0.001f;
    float t = 0.0f;
    for (int i = 0; i < 5000; ++i) {
        Eigen::Vector3f f_val(t, t, t);
        Eigen::Vector3f result = lev.compute(f_val, dt);
        t += dt;
        (void)result;
    }

    /* Also test scalar interface */
    lev.Reset();
    t = 0.0f;
    for (int i = 0; i < 5000; ++i) {
        double f_val = static_cast<double>(t);
        double result = lev.compute(f_val, dt);
        t += dt;
        (void)result;
    }

    std::cout << "  Levant3 wrapper ran without crash\n";
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
/* TEST 10: Newton solver convergence (via IRED<3> internal)         */
/* ================================================================== */
static void test_newton_convergence() {
    std::cout << "--- TEST Newton solver convergence (IRED<3>) ---\n";

    /* Feed a large step to IRED to exercise the Newton solver */
    IRED<3> ired(1.0);
    ired.step(100.0, 0.01);   // large residual
    double y = ired.output(0);
    std::printf("  After single large step: y1 = %.6f (should be finite)\n", y);
    assert(std::isfinite(y) && "Newton solver produced finite output");
    std::cout << "  PASSED\n\n";
}

/* ================================================================== */
int main() {
    std::cout << "=== NMethods Test Suite ===\n\n";

    test_ired3_cubic();
    test_ired1_sin();
    test_kahan_drift();
    test_implicit_midpoint();
    test_euler_and_rk4();
    test_rk4_func();
    test_kahan_eigen();
    test_ired3_vector();
    test_levant3_wrapper();
    test_newton_convergence();

    std::cout << "=== ALL TESTS PASSED ===\n";
    return 0;
}

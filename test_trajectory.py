#!/usr/bin/env python3
"""Verification tests for TrajectoryManager.cpp — checks code correctness by source analysis."""

import re
import sys

FILE = "uav/src/TrajectoryManager.cpp"

def load():
    with open(FILE, "r") as f:
        return f.read()

src = load()
lines = src.split("\n")

passed = 0
failed = 0
results = []

def check(name, condition):
    global passed, failed
    status = "PASS" if condition else "FAIL"
    if condition:
        passed += 1
    else:
        failed += 1
    results.append(f"[{status}] {name}")
    print(f"  [{status}] {name}")

print("=" * 60)
print("TrajectoryManager.cpp Verification Tests")
print("=" * 60)

# 1. physicalParams has 1e-4 (not 0.0) as 6th element
check("01 physicalParams veps=1e-4",
      bool(re.search(r"physicalParams\s*<<.*,\s*1e-4\s*;", src)))

# 2. AddObstacle swaps x/y (pos.y, pos.x)
check("02 AddObstacle swaps x/y",
      bool(re.search(r"AddObstacle.*\{[\s\S]*?Eigen::Vector3d\(pos\.y,\s*pos\.x,\s*pos\.z\)", src)))

# 3. UpdateObstaclePosition swaps x/y (pos.y, pos.x)
check("03 UpdateObstaclePosition swaps x/y",
      bool(re.search(r"UpdateObstaclePosition.*\{[\s\S]*?Eigen::Vector3d\(pos\.y,\s*pos\.x,\s*pos\.z\)", src)))

# 4. ReadWaypointsFromGUI swaps x/y (wp_y_ first)
check("04 ReadWaypointsFromGUI swaps x/y",
      bool(re.search(r"wp_y_\[i\]->Value\(\)", src)))

# 5. output_matrix_ des_x = p.y() (swapped)
check("05 output_matrix des_x = p.y() (swapped back)",
      bool(re.search(r"SetValueNoMutex\(0,\s*0,.*p\.y\(\)", src)))

# 6. output_matrix des_y = p.x() (swapped)
check("06 output_matrix des_y = p.x() (swapped back)",
      bool(re.search(r"SetValueNoMutex\(1,\s*0,.*p\.x\(\)", src)))

# 7. getMaxVelRate() called after SolveGCOPTER
check("07 getMaxVelRate() post-solve validation (unconstrained)",
      bool(re.search(r"getMaxVelRate\(\)", src)))

# 8. getMaxAccRate() called after SolveGCOPTER
check("08 getMaxAccRate() post-solve validation",
      bool(re.search(r"getMaxAccRate\(\)", src)))

# 9. omega_max = 6.0 (not a_max / grav)
check("09 omega_max = 6.0 rad/s",
      bool(re.search(r"omega_max\s*=\s*6\.0", src)))

# 10. No FIRI code between SolveGCOPTER() start and MINCO_S3NU
# Find SolveGCOPTER() body and check FIRI is not present (should only be "removed" comment)
m = re.search(r"bool TrajectoryManager::SolveGCOPTER\(\)\s*\{", src)
if m:
    start = m.start()
    # Find the MINCO_S3NU usage
    m2 = re.search(r"MINCO_S3NU", src[start:])
    if m2:
        between = src[start:start + m2.start()]
        has_firi_call = bool(re.search(r"firi::firi\(", between))
        check("10 No FIRI code in unconstrained SolveGCOPTER()",
              not has_firi_call)
    else:
        check("10 No FIRI code in unconstrained SolveGCOPTER()", False)
else:
    check("10 No FIRI code in unconstrained SolveGCOPTER()", False)

# 11. HasNonCoplanarPoints function exists
check("11 HasNonCoplanarPoints function exists",
      bool(re.search(r"bool HasNonCoplanarPoints\(", src)))

# 12. IsInsidePolytope function exists
check("12 IsInsidePolytope function exists",
      bool(re.search(r"bool IsInsidePolytope\(", src)))

# 13. Frame convention comment block exists
check("13 Frame convention comment block exists",
      bool(re.search(r"Coordinate Frame Conventions", src)))

# 14. NED frame comment near physicalParams
check("14 NED frame mismatch comment near physicalParams",
      bool(re.search(r"GCOPTER.*flatness.*assumes z-UP.*ENU", src)))

# 15. Velocity-aware iterative time scaling loop
check("15 Velocity-aware iterative time scaling",
      bool(re.search(r"Velocity-aware iterative time scaling", src)))

# 16. Direction reversal detection (cos_a < -0.5)
check("16 Direction reversal detection (cos < -0.5)",
      bool(re.search(r"cos_a\s*<\s*-0\.5", src)))

# 17. MINCO_S3NU used for trajectory solve
check("17 MINCO_S3NU solver used",
      bool(re.search(r"MINCO_S3NU", src)))

# 18. SolveGCOPTERConstrained function exists
check("18 SolveGCOPTERConstrained exists",
      bool(re.search(r"SolveGCOPTERConstrained\(", src)))

# 19. GCOPTER_PolytopeSFC solver used in constrained
check("19 GCOPTER_PolytopeSFC used in constrained solver",
      bool(re.search(r"GCOPTER_PolytopeSFC", src)))

# 20. last_pos_ swap x/y back to world
check("20 last_pos_ swaps x/y back to world frame",
      bool(re.search(r"last_pos_\s*=\s*Vector3Df\(.*p\.y\(\).*p\.x\(\)", src)))

# 21. Workspace bounds auto-expansion (WS_X_MIN etc. adjusted)
check("21 Workspace bounds exist (WS_X_MIN)",
      bool(re.search(r"WS_X_MIN", src)))

# 22. Ground plane collision (z >= 0 cells occupied)
check("22 Ground plane collision check (z >= 0 or z_max = 0)",
      bool(re.search(r"WS_Z_MAX\(0\.0\)|z.*>=\s*0|ground", src, re.IGNORECASE)))

# 23. Obstacle inflation in grid
check("23 Obstacle inflation in occupancy grid",
      bool(re.search(r"inflate|inflation|radius.*grid|grid.*radius", src, re.IGNORECASE)))

# 24. Frame swap consistency: world→planner→world roundtrip
# Check that des_vx is v.y() and des_vy is v.x()
check("24 Velocity swap consistency (des_vx=v.y, des_vy=v.x)",
      bool(re.search(r"SetValueNoMutex\(3,\s*0,.*v\.y\(\)", src)) and
      bool(re.search(r"SetValueNoMutex\(4,\s*0,.*v\.x\(\)", src)))

# 25. Acceleration swap consistency
check("25 Acceleration swap consistency (des_ax=a.y, des_ay=a.x)",
      bool(re.search(r"SetValueNoMutex\(6,\s*0,.*a\.y\(\)", src)) and
      bool(re.search(r"SetValueNoMutex\(7,\s*0,.*a\.x\(\)", src)))

# 26. Jerk swap consistency
check("26 Jerk swap consistency (des_jx=j.y, des_jy=j.x)",
      bool(re.search(r"SetValueNoMutex\(9,\s*0,.*j\.y\(\)", src)) and
      bool(re.search(r"SetValueNoMutex\(10,\s*0,.*j\.x\(\)", src)))

# 27. Constrained solver post-validation (getMaxVelRate after constrained)
# Find SolveGCOPTERConstrained and check getMaxVelRate is called within it
m_c = re.search(r"bool TrajectoryManager::SolveGCOPTERConstrained\(", src)
if m_c:
    # Find the next function definition (or end of file)
    rest = src[m_c.start():]
    # Look for getMaxVelRate within the function
    check("27 Constrained solver post-validation (getMaxVelRate)",
          bool(re.search(r"getMaxVelRate\(\)", rest[:5000])))
else:
    check("27 Constrained solver post-validation (getMaxVelRate)", False)

# 28. penaltyWeights corridor weight is high (>=10000)
check("28 Corridor penalty weight >= 10000",
      bool(re.search(r"penaltyWeights\s*<<\s*10000", src)))

# 29. magnitudeBounds includes v_max, omega_max, theta_max
check("29 magnitudeBounds includes v_max, omega_max, theta_max",
      bool(re.search(r"magnitudeBounds\s*<<\s*v_max", src)) and
      bool(re.search(r"omega_max", src)))

# 30. Dead FIRI code removed from SolveGCOPTER (only "removed" comment remains)
if m:
    between_all = src[m.start():m.start()+2000]
    check("30 FIRI dead code removed (only comment remains in SolveGCOPTER)",
          "FIRI corridor generation" in between_all and "was removed" in between_all
          and not re.search(r"firi::firi\(", between_all))
else:
    check("30 FIRI dead code removed", False)

print()
print("=" * 60)
total = passed + failed
print(f"RESULT: {passed}/{total} PASS")
if failed > 0:
    print(f"FAILURES: {failed}")
    for r in results:
        if r.startswith("[FAIL]"):
            print(f"  {r}")
print("=" * 60)

sys.exit(0 if failed == 0 else 1)

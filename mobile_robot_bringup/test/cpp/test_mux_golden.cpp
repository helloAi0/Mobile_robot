// Reads golden vectors written by tools/gen_mux_golden.py (computed by the Python mux) and checks that mux_core.hpp gives the
// same numbers. Usage: test_mux_golden <golden.txt>. Exit code 0 = all cases match.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "mobile_robot_bringup/mux_core.hpp"

static bool same(double a, double b, double tol = 1e-9)
{
  if (std::isnan(a) && std::isnan(b)) {return true;}
  if (std::isinf(a) || std::isinf(b)) {return a == b;}
  return std::fabs(a - b) <= tol;
}

static double num(std::istringstream & in)
{
  std::string t;
  in >> t;
  return std::strtod(t.c_str(), nullptr);
}

int main(int argc, char ** argv)
{
  if (argc < 2) {std::cerr << "usage: test_mux_golden golden.txt\n"; return 2;}
  std::ifstream f(argv[1]);
  std::string line;
  long cases[4] = {0, 0, 0, 0}, bad[4] = {0, 0, 0, 0};
  while (std::getline(f, line)) {
    std::istringstream in(line);
    std::string kind;
    in >> kind;
    if (kind == "FD") {
      const std::size_t n = static_cast<std::size_t>(num(in));
      const double amin = num(in), inc = num(in), cone = num(in), rmax = num(in), center = num(in), expected = num(in);
      std::vector<float> r(n);
      for (auto & v : r) {v = static_cast<float>(num(in));}
      const double got = mux::front_distance(r.data(), n, amin, inc, cone, rmax, center);
      mux::ConeFinder finder(cone, center, rmax);                    // the cached version must agree, also when called twice
      const double got2 = finder(r.data(), n, amin, inc, rmax), got3 = finder(r.data(), n, amin, inc, rmax);
      ++cases[0];
      if (!same(got2, expected) || !same(got3, expected)) {++bad[0]; if (bad[0] < 4) {std::cerr << "ConeFinder mismatch: got " << got2 << " expected " << expected << "\n";}}
      if (!same(got, expected)) {++bad[0]; if (bad[0] < 4) {std::cerr << "FD mismatch: got " << got << " expected " << expected << "\n";}}
    } else if (kind == "GU") {
      mux::Params p;
      const double lin = num(in), ang = num(in), front = num(in);
      const bool ok = num(in) != 0, req = num(in) != 0;
      p.stop_dist = num(in); p.slow_dist = num(in); p.rear_stop_dist = num(in); p.require_scan = req;
      const double rear = num(in), el = num(in), ea = num(in);
      std::string reason;
      in >> reason;
      if (reason == "-") {reason = "";}
      const auto g = mux::guard(lin, ang, front, ok, p, rear);
      ++cases[1];
      if (!same(g.lin, el) || !same(g.ang, ea) || reason != g.reason) {
        ++bad[1];
        if (bad[1] < 4) {std::cerr << "GU mismatch: got " << g.lin << "," << g.ang << "," << g.reason << " expected " << el << "," << ea << "," << reason << "\n";}
      }
    } else if (kind == "LS") {
      mux::Params p;
      mux::Command cur{num(in), num(in)}, tgt{num(in), num(in)};
      const double dt = num(in);
      const bool brake = num(in) != 0;
      p.max_linear = num(in); p.max_angular = num(in); p.max_accel = num(in); p.max_ang_accel = num(in);
      const double el = num(in), ea = num(in);
      const auto c = mux::limit_step(cur, tgt, dt, p, brake);
      ++cases[2];
      if (!same(c.lin, el) || !same(c.ang, ea)) {++bad[2]; if (bad[2] < 4) {std::cerr << "LS mismatch\n";}}
    } else if (kind == "SE") {
      const double now = num(in);
      std::array<double, 3> t{num(in), num(in), num(in)};
      std::array<mux::Command, 3> cmd{};
      for (auto & c : cmd) {c.lin = num(in); c.ang = num(in);}
      std::array<double, 3> to{num(in), num(in), num(in)};
      std::string src;
      in >> src;
      const double el = num(in), ea = num(in);
      std::array<mux::Received, 3> last{};
      for (int i = 0; i < 3; ++i) {last[i].valid = t[i] >= 0; last[i].t = t[i]; last[i].cmd = cmd[i];}
      const auto s = mux::select(last, now, to);
      const char * names[] = {"teleop", "nav", "auto", "none"};
      ++cases[3];
      if (src != names[s.first] || !same(s.second.lin, el) || !same(s.second.ang, ea)) {++bad[3]; if (bad[3] < 4) {std::cerr << "SE mismatch\n";}}
    }
  }
  const char * label[] = {"front_distance", "guard", "limit_step", "select"};
  long total_bad = 0;
  for (int i = 0; i < 4; ++i) {
    std::printf("%-15s %5ld cases, %ld mismatches\n", label[i], cases[i], bad[i]);
    total_bad += bad[i];
    if (cases[i] == 0) {total_bad += 1;}
  }
  return total_bad == 0 ? 0 : 1;
}

// Latency of one mux cycle (what runs every 50 ms plus what runs on every lidar scan), C++ side.
// Prints one JSON line. The Python side is measured by tools/mux_latency.py with the same inputs.
//   mux_bench [iterations]
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

#include "mobile_robot_bringup/mux_core.hpp"

using Clock = std::chrono::steady_clock;

static double pct(std::vector<double> & v, double q)
{
  std::sort(v.begin(), v.end());
  return v[static_cast<std::size_t>(q * (v.size() - 1))];
}

int main(int argc, char ** argv)
{
  const int iters = argc > 1 ? std::atoi(argv[1]) : 20000;
  const std::size_t n = 720;
  const double amin = -3.14159, inc = 2 * 3.14159 / (n - 1);
  std::mt19937 gen(1);
  std::uniform_real_distribution<float> u(0.2f, 9.0f);
  std::vector<float> ranges(n);
  for (auto & r : ranges) {r = u(gen);}
  mux::Params p;
  mux::Command cur{0.0, 0.0};
  std::array<mux::Received, 3> last{};
  last[0] = {true, 10.0, {0.4, 0.2}};
  const std::array<double, 3> to{0.6, 0.5, 0.5};
  mux::ConeFinder front_cone(28.0, 0.0), rear_cone(28.0, 180.0);
  std::vector<double> scan_us, tick_us;
  volatile double sink = 0.0;
  for (int i = 0; i < iters; ++i) {
    ranges[static_cast<std::size_t>(i) % n] = u(gen);            // the scan is never identical twice
    auto t0 = Clock::now();
    const double front = front_cone(ranges.data(), n, amin, inc);
    const double rear = rear_cone(ranges.data(), n, amin, inc);
    auto t1 = Clock::now();
    const auto sel = mux::select(last, 10.2, to);
    const auto g = mux::guard(sel.second.lin, sel.second.ang, front, true, p, rear);
    cur = mux::limit_step(cur, {g.lin, g.ang}, 0.05, p, mux::is_braking(g.reason));
    auto t2 = Clock::now();
    sink = sink + cur.lin + front + rear;
    scan_us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    tick_us.push_back(std::chrono::duration<double, std::micro>(t2 - t1).count());
  }
  std::printf("{\"impl\": \"cpp\", \"iterations\": %d, \"scan_front_rear_us\": {\"median\": %.2f, \"p99\": %.2f, \"max\": %.2f}, "
              "\"tick_us\": {\"median\": %.3f, \"p99\": %.3f, \"max\": %.2f}}\n",
              iters, pct(scan_us, 0.5), pct(scan_us, 0.99), *std::max_element(scan_us.begin(), scan_us.end()),
              pct(tick_us, 0.5), pct(tick_us, 0.99), *std::max_element(tick_us.begin(), tick_us.end()));
  return 0;
}

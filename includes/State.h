#include <iostream>


struct State {
  State(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};


namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std


#pragma once

#include <iostream>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>


struct State {
  State(int time, int x, int y, int c = 1) : time(time), x(x), y(y), cost{c} {}

  State(const State* other) 
  { // copy constructor
      time = int(other->time);
      x = int(other->x);
      y = int(other->y);
      cost = int(other->cost);
  }

   bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool isSameLocation(const State *s) const 
  {
    if (x == s->x && y == s->y) {return true;}
    else {return false;}
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ":(" << s.x << "," << s.y << "," << s.cost << ")";
  }

  bool operator <(const State& s) const
  {
    return x < s.x && y < s.y;
  }

  bool operator >(const State& s) const
  {
    return x > s.x && y > s.y;
  }

  int time;
  int x;
  int y;
  int cost;
};

// create default constructor
namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.cost);
    return seed;
  }
};
}  // namespace std




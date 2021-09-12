#pragma once
// standard includes
#include <iostream>
#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>


// Lowest-Level Datatype used by all planners
struct State {
  // Constructor
  State(int time, int x, int y, int c = 1) : time(time), x(x), y(y), cost{c} {}

  // copy constructor
  State(const State* other) 
  {
      time = int(other->time);
      x = int(other->x);
      y = int(other->y);
      cost = int(other->cost);
  }

  // overload == operator for state comparison
  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  // check if two States are at the same location
  bool isSameLocation(const State *s) const 
  {
    if (x == s->x && y == s->y) {return true;}
    else {return false;}
  }

  // overload << operator for simple state print out
  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ":(" << s.x << "," << s.y << "," << s.cost << ")";
  }

  // overload < for hashing
  bool operator <(const State& s) const
  {
    return x < s.x && y < s.y;
  }

  int time;
  int x;
  int y;
  int cost;
};

// create default hash constructor
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

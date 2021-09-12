#pragma once
// my includes
#include "../includes/State.h"


// Conflict structure
struct Conflict
{
  // which type of conflict
  enum Type {
    Vertex,
    Edge,
    Explanation
  };
  Type type;  
  // (a_i, a_j, v_i, v_j, T_i, T_j)
  size_t agent1;
  size_t agent2;
  int x1;
  int y1;
  int x2;
  int y2;
  int time1;
  int time2;
  // nice print statements -- type specific
  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) 
  {
    switch (c.type) 
    {
      case Vertex:
        return os << c.time1 << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << "Edge(" << c.time1 << ": (" << c.x1 << "," << c.y1 << "), " << c.time2 << ": (" << c.x2
                  << "," << c.y2 << ")";
      case Explanation:
        return os << "Explanation Conflict: " << std::endl << "Agent: " << c.agent1 << " " << "Time: " << c.time1 << ": Vertex(" << c.x1 << "," << c.y1 << ")" << std::endl
          << "Agent: " << c.agent2 << " " << "Time: " << c.time2 << ": Vertex(" << c.x1 << "," << c.y1 << ")";
    }
    return os;
  }
};

// Vertex Constraint -- agent i cannot be in t: (x, y) 
struct VertexConstraint
{
  // constructor
	VertexConstraint(int agent, int t, int x, int y): 
		agent{agent}, time{t}, x{x}, y{y} {}
	
  // nice print statements
  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& v)
  {
    return os << "Agent: " << v.agent << " Vertex(" << v.time << ": (" << v.x <<
      "," << v.y << "))";
  }
  // save relevent information
	const int agent;
	const int time;
	const int x;
	const int y;
};

// Edge Constraint -- no edge swapping
struct EdgeConstraint
{
  // constructor
	EdgeConstraint(int agent, int time1, int time2, int x1, int y1, int x2, int y2)
      : agent{agent}, time1(time1), time2(time2), x1(x1), y1(y1), x2(x2), y2(y2) {}
  
  // nice print statements
  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& e)
  {
    return os << "Agent: " << e.agent << " Edge(" << e.time1 << ": (" << e.x1 << "," << e.y1 << "), " << e.time2 << ": (" << e.x2
                  << "," << e.y2 << "))";
  }
  // save relevent information
  const int agent;
  const int time1;
	const int x1;
	const int y1;
  const int time2;
	const int x2;
	const int y2;
};

// abstrcted constraint used by planners
struct Constraint
{
  // can be either vertex or edge
  public:
      // add a constraint once it is found
	    void add(VertexConstraint *v) {vertexC = v;};
	    void add(EdgeConstraint *e) {edgeC = e;};
      // get the constraint
	    VertexConstraint* getVertexConstraint() {return vertexC;};
	    EdgeConstraint* getEdgeConstraint() {return edgeC;};
private:
  // save important information
	VertexConstraint *vertexC{nullptr};
	EdgeConstraint *edgeC{nullptr};
};






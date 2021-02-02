#pragma once

#include "../includes/State.h"

//                                               

struct Conflict
{
  enum Type {
    Vertex,
    Edge,
  };

  size_t agent1;
  size_t agent2;
  Type type;

  int time1;
  int x1;
  int y1;
  int time2;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) 
  {
    switch (c.type) 
    {
      case Vertex:
        return os << c.time1 << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << "Edge(" << c.time1 << ": (" << c.x1 << "," << c.y1 << "), " << c.time2 << ": (" << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};


struct VertexConstraint
{
	VertexConstraint(int agent, int t, int x, int y): 
		agent{agent}, time{t}, x{x}, y{y} {}
	// constraints are for single agent (i.e. agent 1 cannot go to a state s at time t)
	const int agent;
	const int time;
	const int x;
	const int y;
};


struct EdgeConstraint
{
	EdgeConstraint(int agent, int time1, int time2, int x1, int y1, int x2, int y2)
      : agent{agent}, time1(time1), time2(time2), x1(x1), y1(y1), x2(x2), y2(y2) {}
    const int agent;
    const int time1;
	const int x1;
	const int y1;
    const int time2;
	const int x2;
	const int y2;
};


struct Constraint
{
public:
	void add(VertexConstraint *v) {vertexC = v;};

	void add(EdgeConstraint *e) {edgeC = e;};

	VertexConstraint* getVertexConstraint() {return vertexC;};

	EdgeConstraint* getEdgeConstraint() {return edgeC;};
private:
	VertexConstraint *vertexC{nullptr};
	EdgeConstraint *edgeC{nullptr};
};
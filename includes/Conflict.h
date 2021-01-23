#pragma once

#include "../includes/State.h"

struct Conflict
{
	Conflict(int agentI, int agentJ, State st): m_agentI(agentI), 
		m_agentJ(agentJ), m_state(st) {}
	// a conflict is between two agents -- need to track them
	const int m_agentI;
	const int m_agentJ;
	// a conflict also tracks a vertex and time -- both contained in state
	const State m_state;
};


struct VertexConstraint
{
	~VertexConstraint();
	VertexConstraint(int agent, State st): 
		m_agent(agent), m_state(st) {}
	// constraints are for single agent (i.e. agent 1 cannot go to a state s at time t)
	const int m_agent{0};
	const State m_state;
};


struct EdgeConstraint
{
	EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : m_time(time), m_x1(x1), m_y1(y1), m_x2(x2), m_y2(y2) {}
    const int m_time;
	const int m_x1;
	const int m_y1;
	const int m_x2;
	const int m_y2;
};


struct Constraint
{
public:
	void add(VertexConstraint *v) {m_vertexC = v;};

	void add(EdgeConstraint *e) {m_edgeC = e;};

	VertexConstraint* getVertexConstraint() {return m_vertexC;};

	EdgeConstraint* getEdgeConstraint() {return m_edgeC;};
private:
	VertexConstraint *m_vertexC{nullptr};
	EdgeConstraint *m_edgeC{nullptr};
};
#pragma once

#include <unordered_set>
#include "Environment.h"
#include "Astar.h"

typedef std::vector<std::vector<State>> Solution;

struct VertexConstraint
{
	VertexConstraint(int time, int x, int y): m_time(time), m_x(x), m_y(y) {}
	const int m_time;
	const int m_x;
	const int m_y;
};

// namespace std {
// template <>
// struct hash<VertexConstraint> {
//   size_t operator()(const VertexConstraint& s) const {
//     size_t seed = 0;
//     boost::hash_combine(seed, s.m_time);
//     boost::hash_combine(seed, s.m_x);
//     boost::hash_combine(seed, s.m_y);
//     return seed;
//   }
// };
// }  // namespace std

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

// namespace std {
// template <>
// struct hash<EdgeConstraint> {
//   size_t operator()(const EdgeConstraint& s) const {
//     size_t seed = 0;
//     boost::hash_combine(seed, s.m_time);
//     boost::hash_combine(seed, s.m_x1);
//     boost::hash_combine(seed, s.m_y1);
//     boost::hash_combine(seed, s.m_x2);
//     boost::hash_combine(seed, s.m_y2);
//     return seed;
//   }
// };
// }  // namespace std

struct Constraints
{
// public:
// 	void add(VertexConstraint *v) {m_vertexCs.insert(v);};

// 	// void add(EdgeConstraint *e) {m_edgeCs.insert(e);};
// private:
	std::unordered_set<VertexConstraint*, std::hash<VertexConstraint*>> m_vertexCs;
	std::unordered_set<EdgeConstraint*, std::hash<EdgeConstraint*>> m_edgeCs;
};


class CBS
{
public:
	CBS(Environment *env);

	bool plan(const std::vector<State>& startStates, Solution& solution);

	Solution lowLevelSearch(const std::vector<State>& startStates, Constraints *c);

private:
	struct conflictNode
	{
	public:

		conflictNode(Solution solution): m_solution(solution)
		{
			m_cost = calcCost();
		};

		int calcCost()
		{
			int cost = 0;
			for (std::vector<State> sol: m_solution)
			{
				cost = cost + sol.size();
			}
			return cost;
		};

		Solution m_solution;
		Constraints *m_constraints{nullptr};
		int m_cost{0};
		conflictNode *parent{nullptr};
	};

	// for open heap
	class myconflictComparator
	{
	public:
    	int operator() (const conflictNode *n1, const conflictNode *n2)
    	{
        	return n1->m_cost > n2->m_cost;
    	}
	};

	// bool evalSolution(conflictNode *n);
private:
	Environment *m_env;
	A_star *m_planner{nullptr};
};

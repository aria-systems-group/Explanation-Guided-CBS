#include <vector>
#include <queue>
#include <unordered_set>


// Define a Plan
template <typename State, typename Action, typename Cost>
struct PlanResult 
{
	// states and their gScore
	std::vector<std::pair<State, Cost> > states;
	// actions and their cost
	std::vector<std::pair<Action, Cost> > actions;
	// actual cost of the result
	Cost cost;
};

// A* Planner
class A_star
{
public:
	A_star(Environment& env);

	bool plan(const State& startState, PlanResult<State, Action, Cost>& solution, Cost initCost = 0);

	// this is the Euclidean distance function
	Cost heuristicFunc(const State& st, const Location& goal);

private:
	struct Node
	{
    	Node(const State& state, Cost gScore, Cost hScore)
        	: state(state), gScore(gScore), hScore(hScore) {}

    	friend std::ostream& operator<<(std::ostream& os, const Node& node)
    	{
    	  os << "state: " << node.state << " fScore: " << (node.gScore + node.hScore);
    	  return os;
    	}
    	// gScore = is the cost of the path from the start node
    	// hScore = heuristic cot function (Equclidean dist from node to goal)
    	State state;
    	Cost gScore;
    	Cost hScore;
	};
	// To compare two Nodes -- f(n) = g(n) + h(n)
	// calculate the best next-node from open heap
	class myComparator
	{
	public:
    	int operator() (const Node& n1, const Node& n2)
    	{
    		Cost fScore1 = n1.gScore + n1.hScore;
    		Cost fScore2 = n2.gScore + n2.hScore;
        	return fScore1 > fScore2;
    	}
	};
private:
	Environment& m_env;
	// min heap for open set
	std::priority_queue <Node, std::vector<Node>, myComparator > m_open_heap;
	std::unordered_set<State, std::hash<State>> m_closed_list;

};




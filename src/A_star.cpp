#include <unordered_set>
#include <queue>
#include "../includes/State.h"
#include "../includes/Location.h"
#include "../includes/Environment.h"
#include "../includes/A_star.h"


A_star::A_star(Environment& env): m_env(env) {};

bool A_star::plan(const State& startState, std::vector<State> &solution, Cost initCost)
{
	  solution.clear();
   	// EXAMPLE SYNTAX
   	// generate a node with different 
   	// Node n1(startState, fcost, gcost);

   	// insert a state into a closed list
   	// m_closed_list.insert(n1.state);

   	// One by one extract items from min heap
   	// std::cout << m_open_heap.top() << std::endl;
    // while (m_open_heap.empty() == false)
    // {
    //     std::cout << m_open_heap.top() << std::endl;
    //     m_open_heap.pop();
    // }

    // begin algorithm
   	// Initialize node at start state -- can keep zero f-cost
   	Node *startNode = new Node(startState, 0, 0);

   	// init neighbors
   	std::vector<Neighbor<State, Action, Cost>> neighbors;

   	// put startState in open list -- can leave f-score at 0
   	m_open_heap.emplace(startNode);
    m_open_list.insert(startNode->state);

   	while (!m_open_heap.empty())
   	{
   		// get lowest cost node
   		Node *current = m_open_heap.top();
   		m_open_heap.pop();

   		// generate neighbors of current node
   		m_env.expandState(current->state, neighbors);
   		// for each successor
   		for (auto& neighbor : neighbors)
      {
        // if neighbor is goal
        if (m_env.isStateGoal(neighbor.state))
        {
        	std::cout << "found solution" << std::endl;
          Cost gScoreNew = current->gScore + 1.0;
          Cost hScoreNew = m_env.heuristicFunc(neighbor.state);
          Cost fScoreNew = gScoreNew + hScoreNew;
          Node *solNode = new Node(neighbor.state, gScoreNew, hScoreNew);
          solNode->parent = current;

          while (solNode != nullptr)
          {
            solution.insert(solution.begin(), solNode->state);
            std::cout << solNode->state << std::endl;
            solNode = solNode->parent;
          }
          std::cout << "exiting planner" << std::endl;
          m_env.updateAgent();
        	return true;
        }

        // if not in open list
        if (m_open_list.find(neighbor.state) == m_open_list.end())
        {
          // if a node same as neighbor in closed, we ignore
          // otherwise, add node to open
          if (m_closed_list.find(neighbor.state) == m_closed_list.end())
          {
            Cost gScoreNew = current->gScore + 1.0;
            Cost hScoreNew = m_env.heuristicFunc(neighbor.state);
            Cost fScoreNew = gScoreNew + hScoreNew;

            // If it isn’t on the open list, add it to 
            // the open list.
            //                OR 
            // If it is on the open list already, check 
            // to see if this path to that square is better, 
            // using 'f' cost as the measure. 
            if (m_open_list.find(neighbor.state) == m_open_list.end() || (current->gScore + current->hScore) > fScoreNew)
            {
              //Make the current square the parent of this square. Record the 
              // f, g, and h costs of the square cell 
              // Node n = Node(neighbor.state, gScoreNew, hScoreNew);

              Node *n = new Node(neighbor.state, gScoreNew, hScoreNew);
              // std::cout << neighbor.state << std::endl;
              // std::cout << current.state << std::endl;
              n->parent = current;
              // std::cout << " i am here now" << std::endl;
              // std::cout << m_open_heap.size() << std::endl;
              m_open_heap.emplace(n);
              m_open_list.insert(n->state);
              // std::cout << m_open_heap.size() << std::endl;   
            }
          }
        }
      }
    m_closed_list.insert(current->state);
  }
  std::cout << "No solution exists." << std::endl;
  m_env.updateAgent();
	return false;
};
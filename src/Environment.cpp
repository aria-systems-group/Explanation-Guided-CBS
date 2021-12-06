#include "../includes/Environment.h"


// find valid neighbors of a state
void Environment::expandState(const State *st, std::vector<State*>& neighbors, 
    std::vector<Constraint*> constraints, bool isNodeWaiting)
{
    // clear previous data
    neighbors.clear();

    // init and check "up" state
    State *up = new State(st->time + 1, st->x, st->y + 1);
    // init and check "down" state
    State *down = new State(st->time + 1, st->x, st->y - 1);
    // init and check "right" state
    State *right = new State(st->time + 1, st->x + 1, st->y);
    // init and check "left" state
    State *left = new State(st->time + 1, st->x - 1, st->y);
    // init and add self state
    if (isNodeWaiting)
    {
        State *stay = new State(st->time + 1, st->x, st->y);
        neighbors.push_back(stay);
    }

    if (isStateValid(st, up, constraints))
    {
        neighbors.push_back(up);
    }
    if (isStateValid(st, down, constraints))
    {
        neighbors.push_back(down);
    }
    if (isStateValid(st, right, constraints))
    {
        neighbors.push_back(right);
    }
    if (isStateValid(st, left, constraints))
    {
        neighbors.push_back(left);
    }
}

// Find if state is valid, given constraints
bool Environment::isStateValid(const State *curr, const State *nxt, 
    const std::vector<Constraint*> constraints) const
{
    // is in env bounds
    if ( 0 > nxt->x || nxt->x > m_dimx || 0 > nxt->y || nxt->y > m_dimy)
        return false;
    // is in true obstacles
    for (Location *obs : m_obstacles)
    {
        if (nxt->x == obs->x && nxt->y == obs->y)
            return false;
    }
    // is in tmp obs.
    for (Location *tobs : m_tmp_obs)
    {
        if (nxt->x == tobs->x && nxt->y == tobs->y)
            return false;
    }

    // is in timed obs
    for (auto time_obs: m_timed_obs)
    {
        if ((nxt->time >= time_obs.t_min) && (nxt->time <= time_obs.t_max))
        {
            // need to check obs
            if ((nxt->x == time_obs.x) && (nxt->y == time_obs.y))
            {
                // printf("State is in timed obs! \n");
                return false;
            }
        }
    }

    if (useCollisionChecking)
    {
        // from exising solutions, get a list of states to check
        std::vector<State*> needCheck;
        for (std::vector<State*> sol: m_existingSol)
        {
            if (nxt->time <= sol.back()->time)
                needCheck.push_back(sol[nxt->time]);
        }
        // next, check that needCheck and curr are not the same state
        for (State *st: needCheck)
        {
            if (st->isSameLocation(nxt))
                return false;
        }
    }
    // need to also account for constraints
    // iterate through constraints and see if state matches any, 
    // if so, return false
    for (Constraint *c: constraints)
    {
        // note that State == is overloaded and we already provide 
        // agent relevant constraints
        VertexConstraint *v = c->getVertexConstraint();
        EdgeConstraint *e = c->getEdgeConstraint();
        if (v != nullptr)
        {
            if ( (v->x == nxt->x) && (v->y == nxt->y) && (v->time == nxt->time))
                return false;
        }

        if (e != nullptr)
        {
            if ((e->x1 == curr->x) && (e->y1 == curr->y) && (e->time1 == curr->time))
            {
                if ((e->x2 == nxt->x) && (e->y2 == nxt->y) && (e->time2 == nxt->time))
                {
                    return false;
                }
            }
        }
    }
    return true;
}

void Environment::abstractGraph(const int sz)
{
	// get new x/y step size
	const double x_new = m_dimx / double(sz);
	const double y_new = m_dimy / double(sz);

	// create new graph with new (x,y) pairs
	std::vector<std::pair<const double, const double>> new_graph;
	for (int xn = 0; xn < sz; xn++)
		for (int yn = 0; yn < sz; yn++)
			new_graph.push_back(std::pair((xn * x_new + (x_new / 2)), (yn * y_new + (y_new / 2))));

	// map all points in old graph to a point in new graph
	std::map<std::pair<double, double>, cell> equiv;
	for (int x = 0; x <= m_dimx; x++)
	{
		for (int y = 0; y <= m_dimy; y++)
		{
			vertex old_pt(x, y);
			double dist2v = std::numeric_limits<double>::infinity();
			std::pair<const double, const double> *closestV;
			for (auto& v: new_graph)
			{
				// calc distance
				double newDist = abs (old_pt.first - v.first) + 
     				abs (old_pt.second - v.second);
     			if (newDist < dist2v)
     			{
     				dist2v = newDist;
     				closestV = &v;
     			}
			}
			// map old_pt to closestV
			if (equiv.contains((*closestV)))
				equiv[(*closestV)].insert(old_pt);
			else
			{
				cell new_cell{old_pt};
				equiv.insert({(*closestV), new_cell});
			}
		}
	}
	// check my work
	for(std::map<std::pair<double, double>, cell>::iterator it = equiv.begin(); it != equiv.end(); ++it)
	{
		printf("Showing %lu Vertices mapping to [%0.2f, %0.2f]: \n", 
			(it->second).size(), (it->first).first, (it->first).second);
		for (std::set<vertex>::iterator it2 = (it->second).begin(); 
			it2 != (it->second).end(); ++it2)
		{
			printf("	[%d, %d] \n", it2->first, it2->second);
		}
	}
	// equiv is the new graph used
}

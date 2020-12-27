// #include "../includes/State.h"
// #include "../includes/Location.h"


class Environment {
    public:
        Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                std::vector<Location> goals):
            m_dimx(dimx),
            m_dimy(dimy),
            m_obstacles(std::move(obstacles)),
            m_goals(std::move(goals)),
            m_agentIdx(0)
            {}

        Environment(const Environment&) = delete;
        Environment& operator=(const Environment&) = delete;
    private:
        int m_dimx;
        int m_dimy;
        std::unordered_set<Location> m_obstacles;
        std::vector<Location> m_goals;
        size_t m_agentIdx;
};


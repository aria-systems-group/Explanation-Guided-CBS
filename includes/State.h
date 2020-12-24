#include <iostream>

class State 
{
public:
    const void setX(const int pt)
    {
        x = pt;
    }
    const void setY(const int pt)
    {
        y = pt;
    }
    const void printState(std::ostream & output)
    {
        output << "(" << x << ", " << y << ")" << std::endl;
    }
private:
    int x;
    int y;
};
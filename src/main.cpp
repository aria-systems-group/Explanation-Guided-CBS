#include <iostream>
#include "../includes/State.h"

int main() {
	State *st;
	st->setX(5);
	st->setY(5);
	std::cout << "State address: " << &st << std::endl;
	std::cout << "State point:";
	st->printState(std::cout);
    return 0;
}
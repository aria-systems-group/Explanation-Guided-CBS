#pragma once
#include <chrono>


// Timer for termination conditions
class Timer
{
public:
	// Timer(const double solveT, const std::chrono::time_point<std::chrono::high_resolution_clock> strt): solveTime_{solveT}, start_{strt}{};

	void operator()(const std::chrono::time_point<std::chrono::high_resolution_clock> strt,const double solveT, bool& done, bool& solved)
	{
		while (!done && !solved)
		{
			const auto now = std::chrono::high_resolution_clock::now();
			const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - strt);
			// std::cout << (duration.count() / 1000000.0) << std::endl;
			if ((duration.count() / 1000000.0) < solveT)
				done = false;
			else
				done = true;
		}
		// std::cout << "Time is up!" <<std::endl;
	}
};
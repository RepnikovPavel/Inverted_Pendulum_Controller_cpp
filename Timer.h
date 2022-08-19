#pragma once
#include <chrono>
#include <iostream>

class Timer
{
	using clock_t = std::chrono::high_resolution_clock;
	using microseconds = std::chrono::microseconds;
public:
	Timer()
		: start_(clock_t::now())
	{
	}
	double Stop() 
	{
		const auto finish = clock_t::now();
		const auto us =
			std::chrono::duration_cast<microseconds>
			(finish - start_).count();
		return us / 1000000.0;
	}
	~Timer()
	{

	}
private:
	const clock_t::time_point start_;
};

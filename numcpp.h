#pragma once
#include <vector>
namespace np
{
	std::vector<double> linspace(double start,double stop, unsigned int num)
	{
		double step = (stop-start) / (num - 1);
		std::vector<double> vec(num);
		for (size_t i = 0; i < num; i++)
		{
			vec[i] = start+ i * step;
		}
		return vec;
	}
	std::vector<double> sin(const std::vector<double>& x) 
	{
		std::vector<double> vec(x.size());
		for (size_t i = 0; i < x.size(); i++)
		{
			vec[i] = std::sin(x[i]);
		}
		return vec;
	}

}
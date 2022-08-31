#pragma once
#include <vector>
#include <cmath>
namespace np
{
	std::vector<double> linspace(double start,double stop, size_t num)
	{
		if (num>1)
		{
			double step = (stop - start) / (num - 1);
			std::vector<double> vec(num);
			for (size_t i = 0; i < num; i++)
			{
				vec[i] = start + i * step;
			}
			return vec;
		}
		else if(num==1)
		{
			return std::vector<double>(1,start);
		}
		else 
		{
			return std::vector<double>();
		}

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
#pragma once

namespace SOUTransfer {
	class Linear
	{

	public:
		Linear() {}
		Linear(double _FROM_a, double _FROM_b,double _TO_a, double _TO_b)
		{
			a = (_TO_b - _TO_a) / (_FROM_b - _FROM_a);
			b = _TO_a - _FROM_a * a;
		}
		Linear(const Linear& other)
		{
			a = other.a;
			b = other.b;
		}
		Linear& operator=(const Linear& other)
		{
			a = other.a;
			b = other.b;
			return *this;
		}
		double call(double x)
		{
			return a * x + b;
		}
		double inverse_call(double y)
		{
			return (y - b) / a;
		}

		~Linear() {}
	private:
		double a;
		double b;
	};
}
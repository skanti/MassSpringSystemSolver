#include <unordered_map>
#include <iostream>
#include <random>
#include "Timer.h"
int main() {

	int n = 1 << 10;
	std::mt19937 mt;
	std::uniform_int_distribution<int> dist(0, 1<<30);

	std::vector<int> a(n), b(n);
	std::vector<int64_t> c(n);
	std::unordered_map<int64_t, int> hashmap;
	for (int i = 0; i < n; i++) {
		a[i] = dist(mt);
		b[i] = dist(mt);
		c[i] = ((int64_t)a[i] << 32) + (int64_t)b[i];
		hashmap.insert({c[i], i});
	}

	Timer::start();
	for (int i = 0; i < n; i++)
		hashmap[c[i]];
	Timer::stop();
	std::cout << "timing (ms): " << Timer::get_timing()*1000.0 << std::endl;

	return 0;
}
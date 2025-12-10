
CXX ?= g++
CXXFLAGS ?= -Wall -Wextra -Wconversion -std=c++23


ppk2: ppk2.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -rf ppk2

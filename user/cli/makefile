# simtemp/user/cli/makefile
APP := simtempCLI
SRC := main.cpp

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -pedantic

all: $(APP)

$(APP): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $(SRC)

clean:
	rm -f $(APP)

.PHONY: all clean

all: clean test
test:
	$(CROSS_COMPILE)g++ -o test test.cpp -Isrc -L. -L.. -lpthread -lCANFDNET
clean:
	rm -vf test

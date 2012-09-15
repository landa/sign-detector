#include "CornerLocator.h"

int main(int argc, char *argv[]) {
	if (argc <= 1) {
		printf("usage: sign_detector <file>\n");
		exit(0);
	}
	CornerLocator cl;
	cl.performDetection(argv[1]);
	waitKey(0);
}

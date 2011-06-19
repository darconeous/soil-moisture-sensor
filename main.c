#include "moist.h"
#include "1wire-slave.h"

int main(void) {
	while(1) {
		moist_update();
	}
}

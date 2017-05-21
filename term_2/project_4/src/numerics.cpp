#include <cmath>

double sigmoid(double x) {
	// Taken from http://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick/	
    if (x >= 0) {
        double z = std::exp(-x);
        return 1 / (1 + z) - 0.5;
    } else {
        double z = std::exp(x);
        return z / (1 + z) - 0.5;
    }
}

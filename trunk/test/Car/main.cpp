#include "Learning/radial_basis_function.hpp"
#include "Learning/vector_function.hpp"
#include "CarLearning.h"
#include <iostream>
#include <ctime>

using namespace learn;

double f1(double x, double y)
{
    return 0.5 - 2*(x-y)*(x+y) + x;
}
double f2(double x, double y)
{
    return -0.5 + 2*(x-y)*(x+y) + x;
}
double drand();
//{
//    return rand()/(double)RAND_MAX;
//}

int main1()
{
    srand(clock());
    double sample[18] = { 0.0, 0.0,  
						  0.0, 0.5,  
					      0.0, 1.0,  
						  0.5, 0.0,  
						  0.5, 0.5,  
						  0.5, 1.0,  
						  1.0, 0.0,  
						  1.0, 0.5,  
						  1.0, 1.0 };
    std::vector<std::vector<double> > s;
    for (int i = 0; i < 20; ++i)
    {
        std::vector<double> x;
        x.push_back(drand());
        x.push_back(drand());
        s.push_back(x);
    }

    typedef radial_basis_function<double>                   scalar_function_type;
    typedef vector_function<double, scalar_function_type>   vector_function_type;
    vector_function_type* af = new vector_function_type(2, scalar_function_type(2, 9, 0.3, -100.0, 100.0, sample, sample + 18));
    //FunctionApproximation<double>* af = new LinearApproximation<double>(2, 2, s.size(), 0.3, s);

    for (int i = 0; i < 200; ++i)
    {
        double x[2] = {drand(), drand()};
        double f[2] = {f1(x[0], x[1]), f2(x[0], x[1])};
        af->update(x, x + 2, f, f+2, 0.1);
    }

    double x[2];
    double err = 0;
    int count = 0;
    for (x[0] = -0.1; x[0] < 1.1; x[0] += 0.1)
    {
            for (x[1] = -0.1; x[1] < 1.1; x[1] += 0.1)
            {
                double fVal[2] = {f1(x[0], x[1]), f2(x[0], x[1])};
                double afVal[2];
                af->compute(x, x + 2, afVal);
                std::cout << " f " << fVal[0] << " " << fVal[1] << " af " << afVal[0] << " " << afVal[1] << std::endl;
                count++;
                err += (fVal[0] - afVal[0])*(fVal[0] - afVal[0]) + (fVal[1] - afVal[1])*(fVal[1] - afVal[1]);
            }
    }
    err /= count;
    std::cout << "Error : " << sqrt(err);
    delete af;
    std::cin.get();
    return 0;
}

int main()
{
    Environment env;
    char c = '0';
    do
    {
        double x, v;
        c = '0';
        std::cout << "Select action:\n";
        std::cout << "r - run\nt - test\ns - save\nl - load\nq - quit\n";
        while (c != 'l' && c != 's' && c != 'r' && c != 't' && c != 'q')
        {
            c = std::cin.get();
        }
        switch (c)
        {
        case 'r':
            env.run(40000);
            break;
        case 't':
            std::cout << "x: ";
            std::cin >> x;
            std::cout << "v: ";
            std::cin >> v;
            env.test(x, v);
            break;
        case 's':
            env.save("func.f");
            break;
        case 'l':
            env.load("func.f");
            break;
        };
    } while (c != 'q');
    std::cin.get();
}

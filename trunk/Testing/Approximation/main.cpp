#include <Learning/neural_network.hpp>
#include <Learning/scalar_neural_network.hpp>
#include <Learning/td_lambda_learning.hpp>
#include <boost/random.hpp>
#include <cmath>

#define LEARN_SPEED             0.3
#define NUM_EDUCATION_SAMPLES   20000
#define NUM_LAYERS              3

struct test1
{
    typedef double                                              real_type;
    typedef learn::logistic<real_type>                          activate_function;
    typedef learn::neural_network<real_type, activate_function> learner_type;
    typedef learn::uniform_weight_distributor<real_type>        weight_distributor;

    static const unsigned input_size  = 4;
    static const unsigned output_size = 4;
    static const unsigned test_size   = 160000;

    static real_type min_input()    { return real_type(0.0); }
    static real_type max_input()    { return real_type(1.0); }

    static real_type min_output()   { return real_type(0.0); }
    static real_type max_output()   { return real_type(1.0); }

    test1() 
    {
        learn::make_cascade_neural_network(learner, input_size, output_size, NUM_LAYERS, true);
        learn::distribute_weights( learner, weight_distributor() );
    }
    
    real_type frac(real_type val) const
    {
        return val - floor(val);
    }

    void operator () (real_type input[4], real_type output[4]) const
    {
        output[0] = frac( sin(input[0]) + input[1]*input[2] );
        output[1] = frac( cos(input[1]) * input[2] );
        output[2] = frac( exp(input[0] + input[1]) * input[3] );
        output[3] = frac( pow( (input[3] + input[2])*(input[3] + input[2]), real_type(0.3) ) + input[3] );
    }

    learner_type learner;
};

struct test2
{
    typedef double                                              real_type;
    typedef learn::logistic<real_type>                          activate_function;
    typedef learn::neural_network<real_type, activate_function> learner_type;
    typedef learn::uniform_weight_distributor<real_type>        weight_distributor;

    static const unsigned input_size  = 4;
    static const unsigned output_size = 4;
    static const unsigned test_size   = 160000;

    static real_type min_input()    { return real_type(0.0); }
    static real_type max_input()    { return real_type(1.0); }

    static real_type min_output()   { return real_type(0.0); }
    static real_type max_output()   { return real_type(1.0); }

    test2() 
    {
        learn::make_cascade_neural_network(learner, input_size, output_size, NUM_LAYERS, true);
        learn::distribute_weights( learner, weight_distributor() );
    }

    void operator () (real_type input[4], real_type output[4]) const
    {
        output[0] = sin(input[0] + input[3] * 4.0) * 0.5 + 0.5;
        output[1] = cos(input[1] * input[2]) * 0.5 + 0.5;
        output[2] = (input[0] + 2.0 * input[3] + 5.0 * input[1]) / 8.0;
        output[3] = input[0] * input[2];
    }

    learner_type learner;
};

struct test3
{
    typedef double                                              real_type;
    typedef learn::hyperbolic_tangent<real_type>                activate_function;
    typedef learn::neural_network<real_type, activate_function> learner_type;
    typedef learn::uniform_weight_distributor<real_type>        weight_distributor;

    static const unsigned input_size  = 4;
    static const unsigned output_size = 4;
    static const unsigned test_size   = 160000;

    static real_type min_input()    { return real_type(-1.0); }
    static real_type max_input()    { return real_type(1.0); }

    static real_type min_output()   { return real_type(-1.0); }
    static real_type max_output()   { return real_type(1.0); }

    test3() 
    {
        learn::make_cascade_neural_network(learner, input_size, output_size, NUM_LAYERS, true);
        learn::distribute_weights( learner, weight_distributor() );
    }

    void operator () (real_type input[4], real_type output[4]) const
    {
        
        output[0] = cos(input[0] * input[3] * 2.0);
        output[1] = sin(input[1] + input[2]);
        output[2] = (input[0] + 2.0 * input[3] + 5.0 * input[1]) / 8.0;
        output[3] = (input[0] * input[1] + input[2] * input[3])/2.0;
        
        /*
        output[0] = 1;
        output[1] = input[1];
        output[2] = 1;
        output[3] = -1;
        */
    }

    learner_type learner;
};

struct test4
{
    typedef double                                              real_type;
    typedef learn::generic_function<real_type>                  activate_function;
    typedef learn::neural_network<real_type, activate_function> learner_type;
    typedef learn::uniform_weight_distributor<real_type>        weight_distributor;

    static const unsigned input_size  = 4;
    static const unsigned output_size = 1;
    static const unsigned test_size   = 160000;

    static real_type min_input()    { return real_type(-1.0); }
    static real_type max_input()    { return real_type(1.0); }

    static real_type min_output()   { return real_type(-1e4); }
    static real_type max_output()   { return real_type(1e4); }

    test4() 
    {
        learn::make_cascade_neural_network(learner, input_size, output_size, NUM_LAYERS, true);
        learn::distribute_weights( learner, weight_distributor() );

        // fill up functions
        learn::setup_function( learner, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( learner, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( learner, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( learner, 3, make_generic_function_with_derivative( learn::linear<real_type>() ) );
    }

    void operator () (real_type input[4], real_type output[1]) const
    {
        output[0] = 1.0*sin(input[0] - input[3] - 4*input[2]) - 0.5;
        //output[1] = 30*cos(input[3]*23) + 24;
        //output[2] = 0;
        //output[3] = 8*sin(input[0] + input[1] + 765);
    }

    learner_type learner;
};

struct test5
{
    typedef double                                              real_type;
    typedef learn::generic_function<real_type>                  activate_function;
    typedef learn::neural_network<real_type, activate_function> network_type;
    typedef learn::scalar_neural_network<network_type>          learner_type;
    typedef learn::uniform_weight_distributor<real_type>        weight_distributor;

    static const unsigned input_size  = 4;
    static const unsigned output_size = 1;
    static const unsigned test_size   = 160000;

    static real_type min_input()    { return real_type(-1.0); }
    static real_type max_input()    { return real_type(1.0); }

    static real_type min_output()   { return real_type(-1e4); }
    static real_type max_output()   { return real_type(1e4); }

    test5() 
    {
        network_type network;
        learn::make_cascade_neural_network(network, input_size, output_size, NUM_LAYERS, true);
        learn::distribute_weights( network, weight_distributor() );

        // fill up functions
        learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<real_type>() ) );
        learn::setup_function( network, 3, make_generic_function_with_derivative( learn::linear<real_type>() ) );

        learner = learner_type(network);
    }

    void operator () (real_type input[4], real_type output[1]) const
    {
        output[0] = 1.0*sin(input[0] - input[3] - 4*input[2]);
        //output[1] = 30*cos(input[3]*23) + 24;
        //output[2] = 0;
        //output[3] = 8*sin(input[0] + input[1] + 765);
    }

    learner_type learner;
};

template<int N, typename T>
void make_sample( unsigned value,
                  unsigned dimSize,
                  T        minVal,
                  T        maxVal,
                  T        sample[N] )
{
    unsigned size = 1;
    for (int j = 0; j<N; ++j) 
    {
        unsigned intSample = (value / size) % dimSize;
        size              *= dimSize;
        sample[j]          = minVal + (maxVal - minVal) * (T(intSample) / dimSize);
    }
}
template<typename Test>
void check_error(const Test& test)
{
    // test
    typename Test::real_type averageError  = Test::real_type(0.0);
    typename Test::real_type maxError      = Test::real_type(0.0);
    unsigned                 dimSize       = unsigned( pow(Test::test_size, Test::real_type(1.0) / Test::input_size) );

    typename Test::real_type input[Test::input_size];
    typename Test::real_type output[Test::output_size];
    typename Test::real_type computed[Test::output_size];

    for (int i = 0; i<Test::test_size; ++i)
    {
        make_sample<Test::input_size>(i, dimSize, Test::min_input(), Test::max_input(), input);
        test(input, output);
        test.learner.compute(input, input + Test::input_size, computed);

        typename Test::real_type localError(0.0);
        for (int j = 0; j<Test::output_size; ++j) {
            localError += (output[j] - computed[j]) * (output[j] - computed[j]);
        }
        localError = sqrt(localError);

        maxError        = std::max(maxError, localError);
        averageError   += localError / Test::test_size;
    }

    std::cout << "Max error: "        << maxError << std::endl
              << "Average error: "    << averageError << std::endl;
}

template<>
void check_error<test5>(const test5& test)
{
    typedef test5 Test;

    // test
    Test::real_type averageError  = Test::real_type(0.0);
    Test::real_type maxError      = Test::real_type(0.0);
    unsigned        dimSize       = unsigned( pow(Test::test_size, Test::real_type(1.0) / Test::input_size) );

    Test::real_type input[Test::input_size];
    Test::real_type output[Test::output_size];
    Test::real_type computed;

    for (int i = 0; i<Test::test_size; ++i)
    {
        make_sample<Test::input_size>(i, dimSize, Test::min_input(), Test::max_input(), input);
        test(input, output);
        computed = test.learner.compute(input, input + Test::input_size);

        Test::real_type localError(0.0);
        for (int j = 0; j<Test::output_size; ++j) {
            localError += (output[j] - computed) * (output[j] - computed);
        }
        localError = sqrt(localError);

        maxError        = std::max(maxError, localError);
        averageError   += localError / Test::test_size;
    }

    std::cout << "Max error: "        << maxError << std::endl
              << "Average error: "    << averageError << std::endl;
}
#include <valarray>
int main()
{
    typedef test5 Test;


    // learner
    Test test;

    std::cout << "Learn speed: "            << LEARN_SPEED << std::endl
              << "Num education samples: "  << NUM_EDUCATION_SAMPLES << std::endl
              << "Test granularity: "       << Test::test_size << std::endl
              << "=====================================\n";

    std::cout << "No education:\n";
    check_error(test);
    std::cout << "=====================================\n";

    //Test::real_type computed[Test::output_size];

    size_t numSamples = 0;
    Test::real_type step_size = LEARN_SPEED;
    while (true)
    {
        {
            using namespace learn;

            boost::mt19937 rng;
            boost::uniform_real<Test::real_type> dist( Test::min_input(), Test::max_input() );
            boost::variate_generator< boost::mt19937&,
                                      boost::uniform_real<Test::real_type> > generator(rng, dist);
            Test::real_type averageError  = Test::real_type(0.0);
            Test::real_type maxError      = Test::real_type(0.0);

            // educate
            Test::real_type input[Test::input_size];
            Test::real_type output[Test::output_size];
            for (int i = 0; i<NUM_EDUCATION_SAMPLES; ++i)
            {
                for (int j = 0; j<Test::input_size; ++j) {
                    input[j] = generator();
                }

                test(input, output);

                Test::real_type computed = test.learner.compute(input, input + Test::input_size);
                ublas::vector<Test::real_type> gradient(test.learner.num_params());
                test.learner.get_gradient(input, input + Test::input_size, gradient.begin());

                ublas::vector<Test::real_type> params(test.learner.num_params());
                test.learner.get_params(params.begin());
                params += step_size*(output[0] - computed)*gradient;
                test.learner.set_params(params.begin(), params.end());
                if (step_size > 0.01) step_size *= 0.999;

                //test.learner.update(input, input + Test::input_size, output[0], LEARN_SPEED);

                //test.learner.compute(input, input + Test::input_size, computed);
                //test.learner.update( input, 
                //                     input + Test::input_size, 
                //                     output, 
                //                     output + Test::output_size, 
                //                     LEARN_SPEED );

                //Test::real_type localError(0.0);
                //for (int j = 0; j<Test::output_size; ++j) {
                //    localError += (output[j] - computed[j]) * (output[j] - computed[j]);
                //}
                //localError = sqrt(localError);

                //maxError        = std::max(maxError, localError);
                //averageError   += localError / NUM_EDUCATION_SAMPLES;
            }
            std::cout << "Num training samples: " << numSamples << std::endl;
            std::cout << "On lrealning samples\n Max error: "        << maxError << std::endl
                      << "Average error: "    << averageError << std::endl;
            numSamples += NUM_EDUCATION_SAMPLES;
        }


        check_error(test);
        std::cout << "=====================================\n";
    }

    return 0;
}

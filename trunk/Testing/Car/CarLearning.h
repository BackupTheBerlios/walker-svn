#ifndef __TESTING_CAR_LEARNING_H___
#define __TESTING_CAR_LEARNING_H___
#include "Learning/radial_basis_function.hpp"
#include "Learning/reinforcement_learning.hpp"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/scoped_array.hpp>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include "Learning/serialization.hpp"
#include "Learning/td_lambda_learning.hpp"
#include "Learning/stub_learning.hpp"
#include "Learning/scalar_neural_network.hpp"
#include "Learning/plain_direct_learning.hpp"

using namespace learn;
using namespace boost::numeric::ublas;

const double minX = -1.2, maxX = 0.5;
const double minV = -0.7, maxV = 0.7;

double drand()
{
    return rand()/(double)RAND_MAX;
}

double bound(double x, double min, double max)
{
    return (x < min)? min : (x > max)? max : x;
}

template<class Analyzer>
class Car
{
public:
    typedef double  value_type;

public:
    Car(){}
    Car(const Analyzer& _analyzer) :
        analyzer(_analyzer)
    {
        initialise();
    }
    void initialise()
    {
        initialise( drand()*(maxX - minX) + minX,
                    drand()*(maxV - minV) + minV );
    }

    void initialise(double x, double v)
    {
        state[0] = x;
        state[1] = v;
        terminal = false;
    }

    // Implement Environment
    size_t  state_size() const  { return 2; }
    size_t  action_size() const { return 1; }

    template<typename InIterator>
    double perform_action( InIterator     firstPin,
                           InIterator     endPin )
    {
        double a = bound(*firstPin, -1, 1);

        state[0] = bound(state[0] + state[1]/10, minX, maxX);
        state[1] = bound(10*(state[1]/10 + 0.001*a - 0.0025*cos(3*state[0])), minV, maxV);
        if (state[0] == minX)
        {
            state[1] = bound(state[1], 0, maxV);
        }
        if (state[0] == maxX)
        {
            terminal = true;
        }

        return analyzer(*this);
    }

    template<typename OutIterator>
    bool query_state(OutIterator stateOut)
    {
        std::copy(state, state + 2, stateOut);
        return terminal;
    }

public:
    Analyzer    analyzer;
    bool        terminal;
    double      state[2];
};

struct WinConditionAnalyzer
{
    double operator () (const Car<WinConditionAnalyzer>& car) const { return car.terminal ? 0.0 : -1.0; }
};

class Environment
{
public:
    typedef Car<WinConditionAnalyzer>                       environment;

    typedef radial_basis_function<double>                   scalar_function_type;
    typedef vector_function<double, scalar_function_type>   vector_function_type;
    //typedef learn::neural_network< float, 
    //                               learn::generic_function<float> >     network_type;
    //typedef learn::scalar_neural_network<network_type>             scalar_function_type;

    //typedef learn::neural_network< float, 
    //                               learn::hyperbolic_tangent<float> >   vector_function_type;

    //typedef learn::td_lambda_learning< double,
    //                                   environment&,
    //                                   scalar_function_type,
    //                                   vector_function_type >       reinforcement_learning;
    typedef learn::plain_direct_learning< double,
                                       environment&,
                                       vector_function_type >       reinforcement_learning;
    typedef boost::shared_ptr<reinforcement_learning>               reinforcement_learning_ptr;

public:
    Environment()
    {

        std::vector<double> sample;
        for (double x = minX - 0.4; x <= maxX + 0.4; x += 0.2)
        {
            for (double v = minV - 0.4; v <= maxV + 0.4; v += 0.2)
            {
                sample.push_back(x);
                sample.push_back(v);
            }
        }
        env = environment( WinConditionAnalyzer() );
        //scalar_function_type scalarFunction = scalar_function_type(2, sample.size(), 0.1, -1000, 1000, sample);
        vector_function_type vectorFunction = vector_function_type(1, scalar_function_type(2, sample.size() / 2, 0.1, -1, 1, sample.begin(), sample.end()));

        learner.reset( new reinforcement_learning( env, vectorFunction));


        //env = environment( WinConditionAnalyzer() );
        //network_type network;
        //learn::make_cascade_neural_network(network, 2, 1, 2, true, 10000.0f, -10000.0f);
        ////learn::distribute_weights( network, learn::uniform_weight_distributor<float>() );

        //// fill up functions
        //learn::setup_function( network, 0, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
        //learn::setup_function( network, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
        ////learn::setup_function( network, 2, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
        //learn::setup_function( network, 2, make_generic_function_with_derivative( learn::linear<float>() ) );
        //
        //scalar_function_type scalarFunction(network);
        //vector_function_type vectorFunction;
        //learn::make_cascade_neural_network(vectorFunction, 2, 1, 3, true);
        ////learn::distribute_weights( vectorFunction, learn::uniform_weight_distributor<float>() );

        //learner.reset( new reinforcement_learning( env, 
        //                                                scalarFunction, 
        //                                                vectorFunction ) );
    }

    void run(int numSteps)
    {
		learner->new_episode();
        for (int i = 0; i < numSteps; ++i)
        {
            step();
        }
		//std::cout << learner->numEpisodes << std::endl;
    }

    void step()
    {
        environment& e = learner->get_environment();
        size_t stateSize = e.state_size();

        boost::scoped_array<double> state( new double[stateSize] );
        bool terminal = e.query_state( state.get() );

        if (terminal)
        {
            learner->update();
            e.initialise();
            learner->new_episode();
        }
        else
        {
            learner->update();
        }
    }

    void test(double x, double v)
    {
        environment& e = learner->get_environment();
        e.initialise(x, v);
        int counter = 0;
        while ( !e.terminal )
        {
            size_t stateSize = e.state_size();
            boost::scoped_array<double> state( new double[stateSize] );
            e.query_state( state.get() );

            double a, v;
            learner->get_best_action_function().compute(state.get(), state.get() + stateSize, &a);
            //v = learner->get_value_function().compute(state.get(), state.get() + stateSize);
            std::cout << "x: " << state[0] << " v: " << state[1] << "a: " << a /*<< " v(s): "<< v */<< std::endl;

            e.perform_action(&a, &a + 1);
            counter++;
        }
        std::cout << "terminal state. num steps: " << counter << std::endl;
    }
    void save(const char* fileName)
    {/*
        std::ofstream out(fileName);
        out << learner->get_best_action_function();
        out << learner->get_value_function();*/
    }
    void load(const char* fileName)
    {/*
        std::ifstream input(fileName);
        input >> learner->get_best_action_function();
        input >> learner->get_value_function();*/
    }

private:
    reinforcement_learning_ptr  learner;
    environment env;
    //scalar_function_type valueFunc;
    //vector_function_type bestActionFunc;
};

#endif // __TESTING_CAR_LEARNING_H___

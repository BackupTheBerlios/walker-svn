#include "Learning/neural_network.hpp"
#include "Learning/linear_function.hpp"
#include "Learning/serialization.hpp"
#include "Learning/vector_function.hpp"
#include <sstream>

#define BOOST_TEST_MODULE SerializationTest
#include <boost/test/unit_test.hpp>

template<typename T>
bool equal(T val0, T val1, T tolerance)
{
    return abs(val0 - val1) < tolerance;
}

template<typename Functor>
bool compare(const learn::function_with_derivative<Functor>& func0,
                      const learn::function_with_derivative<Functor>& func1)
{
    return func0 == func1;
}

class nil 
{
public:
    template<typename Object>
    bool compare(Object* a, Object* b)
    {
        return false;
    }
};

template<typename First, typename Second>
class cons
{
public:
    typedef First   first;
    typedef Second  second;

    template<typename Object>
    bool compare(Object* a, Object* b)
    {
        first* sa = dynamic_cast<first*>(a);
        first* sb = dynamic_cast<first*>(b);
        
        if (sa && !sb || sb && !sa) {
            return false;
        }
        else if (sa && sb) {
            return (*sa) == (*sb);
        }
        else 
        {
            second s;
            return s.compare(a, b);
        }
    }
};

template<typename T>
bool compare(const learn::function_with_derivative< learn::generic_function<T> >& func0,
             const learn::function_with_derivative< learn::generic_function<T> >& func1)
{
    using namespace learn;

    if (!func0.func && !func1.func) {
        return true;
    }

    typedef abstract_functor< function_with_derivative< logistic<T> > >             logistic_functor;
    typedef abstract_functor< function_with_derivative< hyperbolic_tangent<T> > >   hyperbolic_tangent_functor;
    typedef abstract_functor< function_with_derivative< linear<T> > >               linear_functor;

    cons< logistic_functor
          , cons<hyperbolic_tangent_functor
          , cons<linear_functor
          , nil > > > comparer;

    return comparer.compare( func0.func.get(), func1.func.get() );
}

template<typename T>
bool compare(const learn::generic_multi_function<T>& func0,
             const learn::generic_multi_function<T>& func1)
{
    using namespace learn;

    if (!func0.func && !func1.func) {
        return true;
    }

    typedef abstract_multi_functor< learn::select<T> >    select_functor;
    typedef abstract_multi_functor< constant<T> >         constant_functor;
    typedef abstract_multi_functor< sum<T> >              sum_functor;
    typedef abstract_multi_functor< select_product<T> >   select_product_functor;
    typedef abstract_multi_functor< gaussian<T> >         gaussian_functor;

    cons< select_functor
          , cons<constant_functor
          , cons<sum_functor
          , cons<select_product_functor
          , cons<gaussian_functor
          , nil > > > > > comparer;

    return comparer.compare( func0.func.get(), func1.func.get() );
}

template<typename ValueType, typename ActivateFunction>
bool compare(const learn::neural_network<ValueType, ActivateFunction>&   nn0,
             const learn::neural_network<ValueType, ActivateFunction>&   nn1,
             ValueType                                                   tolerance)
{
    typedef learn::neural_network<ValueType, ActivateFunction> nn_type;

    if ( nn0.num_layers() != nn1.num_layers() ) {
        return false;
    }

    {
        typename nn_type::layer_const_iterator iter0 = nn0.first_layer();
        typename nn_type::layer_const_iterator iter1 = nn1.first_layer();
        for (;
             iter0 != nn0.end_layer();
             ++iter0, ++iter1)
        {
            if ( std::distance( iter0->first_neuron(), nn0.first_neuron() ) != 
                 std::distance( iter1->first_neuron(), nn1.first_neuron() ) ) 
            {
                return false;
            }
            
            if ( std::distance( iter0->end_neuron(), nn0.first_neuron() ) != 
                 std::distance( iter1->end_neuron(), nn1.first_neuron() ) ) 
            {
                return false;
            }
        }
    }

    if ( nn0.num_neurons() != nn1.num_neurons() ) {
        return false;
    }

    {
        typename nn_type::neuron_const_iterator iter0 = nn0.first_neuron();
        typename nn_type::neuron_const_iterator iter1 = nn1.first_neuron();
        for (;
             iter0 != nn0.end_neuron();
             ++iter0, ++iter1)
        {
            if ( std::distance( iter0->first_link(), nn0.first_link() ) != 
                 std::distance( iter1->first_link(), nn1.first_link() ) ) 
            {
                return false;
            }

            if ( std::distance( iter0->end_link(), nn0.first_link() ) != 
                 std::distance( iter1->end_link(), nn1.first_link() ) ) 
            {
                return false;
            }

            if ( !equal(iter0->input, iter1->input, tolerance)
                 || !equal(iter0->output, iter1->output, tolerance)
                 || !equal(iter0->gradient, iter1->gradient, tolerance)
                 || !equal(iter0->errorGradient, iter1->errorGradient, tolerance)
                 || !compare(iter0->activateFunc, iter1->activateFunc) )
            {
                return false;
            }
        }
    }

    if ( nn0.num_links() != nn1.num_links() ) {
        return false;
    }

    {
        typename nn_type::link_const_iterator iter0 = nn0.first_link();
        typename nn_type::link_const_iterator iter1 = nn1.first_link();
        for (;
             iter0 != nn0.first_link();
             ++iter0, ++iter1)
        {
            if ( std::distance( iter0->source_neuron(), nn0.first_neuron() ) != 
                 std::distance( iter1->source_neuron(), nn1.first_neuron() ) ) 
            {
                return false;
            }

            if ( std::distance( iter0->target_neuron(), nn0.first_neuron() ) != 
                 std::distance( iter1->target_neuron(), nn1.first_neuron() ) ) 
            {
                return false;
            }

            if ( !equal(iter0->weight, iter1->weight, tolerance) )
            {
                return false;
            }
        }
    }

    return true;
}

template<typename ValueType, typename FunctionType>
bool compare(const learn::linear_function<ValueType, FunctionType>&   linFunc0,
             const learn::linear_function<ValueType, FunctionType>&   linFunc1,
             ValueType                                                tolerance)
{
    typedef learn::linear_function<ValueType, FunctionType> linear_function_type;
    if ( linFunc0.num_params() != linFunc1.num_params()
         || !equal(linFunc0.regularization(), linFunc1.regularization(), tolerance)
         || !equal(linFunc0.out_min(), linFunc1.out_min(), tolerance)
         || !equal(linFunc0.out_max(), linFunc1.out_max(), tolerance) ) 
    {
        return false;
    }

    // compare parameters
    {
        typename linear_function_type::parameter_const_iterator iter0 = linFunc0.first_parameter();
        typename linear_function_type::parameter_const_iterator iter1 = linFunc1.first_parameter();
        for (;
             iter0 != linFunc0.end_parameter();
             ++iter0, ++iter1)
        {
            if ( !equal(*iter0, *iter1, tolerance) ) {
                return false;
            }
        }

    }

    // compare functions
    {
        typename linear_function_type::function_const_iterator iter0 = linFunc0.first_function();
        typename linear_function_type::function_const_iterator iter1 = linFunc1.first_function();
        for (;
             iter0 != linFunc0.end_function();
             ++iter0, ++iter1)
        {
            if ( !compare(*iter0, *iter1) ) {
                return false;
            }
        }
    }

    return true;
}

template<typename ValueType, typename ScalarFunction>
bool compare(const learn::vector_function<ValueType, ScalarFunction>&   vecFunc0,
             const learn::vector_function<ValueType, ScalarFunction>&   vecFunc1,
             ValueType                                                  tolerance)
{
    typedef learn::vector_function<ValueType, ScalarFunction> vector_function_type;
    if ( vecFunc0.num_components() != vecFunc1.num_components() ) {
        return false;
    }


    // compare components
    {
        typename vector_function_type::function_const_iterator iter0 = vecFunc0.first_function();
        typename vector_function_type::function_const_iterator iter1 = vecFunc1.first_function();
        for (;
             iter0 != vecFunc0.end_function();
             ++iter0, ++iter1)
        {
            if ( !compare(*iter0, *iter1, tolerance) ) {
                return false;
            }
        }
    }

    return true;
}

BOOST_AUTO_TEST_CASE(neural_network_serialization)
{
    // check neural network serialization
    for(int i = 1; i<=10; ++i)
    {
        for (int j = 1; j<=3; ++j)
        {
            learn::neural_network< float, learn::hyperbolic_tangent<float> > nn[2];
            learn::make_cascade_neural_network(nn[0], i, i, j, true);
            learn::distribute_weights( nn[0], learn::uniform_weight_distributor<float>() );

            // check text serialization
            {
                std::ostringstream oss;
                boost::archive::text_oarchive oa(oss);
                oa << nn[0];

                std::istringstream iss( oss.str() );
                boost::archive::text_iarchive ia(iss);
                ia >> nn[1];

                BOOST_REQUIRE( compare( nn[0], nn[1], std::numeric_limits<float>::min() ) );
            }

            // check binary serialization
            {
            }
        }
    }

    // check generic neural network serialization
    for(int i = 1; i<=10; ++i)
    {
        learn::neural_network< float, learn::generic_function<float> > nn[2];
        learn::make_cascade_neural_network(nn[0], i, i, 2, true);
        learn::distribute_weights( nn[0], learn::uniform_weight_distributor<float>() );
        learn::setup_function( nn[0], 0, make_generic_function_with_derivative( learn::logistic<float>() ) );
        learn::setup_function( nn[0], 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
        learn::setup_function( nn[0], 2, make_generic_function_with_derivative( learn::linear<float>() ) );

        // check text serialization
        {
            std::ostringstream oss;
            boost::archive::text_oarchive oa(oss);
            oa << nn[0];

            std::istringstream iss( oss.str() );
            boost::archive::text_iarchive ia(iss);
            ia >> nn[1];

            BOOST_REQUIRE( compare( nn[0], nn[1], std::numeric_limits<float>::min() ) );
        }

        // check binary serialization
        {
        }
    }
}

BOOST_AUTO_TEST_CASE(linear_function_serialization)
{
    // check neural network serialization
    for(int i = 1; i<=10; ++i)
    {
        learn::linear_function<float, learn::generic_multi_function<float> > linFunc[2];

        linFunc[0].set_out_min(-1.0f);
        linFunc[0].set_out_max(1.0f);
        linFunc[0].set_regularization(0.01f);

        linFunc[0].push_function( learn::make_multi_generic( learn::constant<float>(1.0f) ) );
        for (int j = 0; j<i; ++j)
        {
            linFunc[0].push_function( learn::make_multi_generic( learn::select<float>(j) ) );
            for (int k = 0; k<i; ++k)
            {
                linFunc[0].push_function( learn::make_multi_generic( learn::select_product<float>(j, k) ) );
            }
        }

        // check text serialization
        {
            std::ostringstream oss;
            boost::archive::text_oarchive oa(oss);
            oa << linFunc[0];

            std::istringstream iss( oss.str() );
            boost::archive::text_iarchive ia(iss);
            ia >> linFunc[1];

            BOOST_REQUIRE( compare( linFunc[0], linFunc[1], std::numeric_limits<float>::min() ) );
        }

        // check binary serialization
        {
        }
    }
}

BOOST_AUTO_TEST_CASE(vector_function_serialization)
{
    // check vector function serialization
    for(int i = 1; i<=10; ++i)
    {
        typedef learn::neural_network< float, learn::generic_function<float> >  neural_network_type;
        typedef learn::vector_function<float, neural_network_type>              vector_function_type;

        neural_network_type nn;
        learn::make_cascade_neural_network(nn, i, i, 2, true);
        learn::distribute_weights( nn, learn::uniform_weight_distributor<float>() );
        learn::setup_function( nn, 0, make_generic_function_with_derivative( learn::logistic<float>() ) );
        learn::setup_function( nn, 1, make_generic_function_with_derivative( learn::hyperbolic_tangent<float>() ) );
        learn::setup_function( nn, 2, make_generic_function_with_derivative( learn::linear<float>() ) );

        vector_function_type vecFunc[2];
        vecFunc[0] = vector_function_type(i, nn);

        // check text serialization
        {
            std::ostringstream oss;
            boost::archive::text_oarchive oa(oss);
            oa << vecFunc[0];

            std::istringstream iss( oss.str() );
            boost::archive::text_iarchive ia(iss);
            ia >> vecFunc[1];

            BOOST_REQUIRE( compare( vecFunc[0], vecFunc[1], std::numeric_limits<float>::min() ) );
        }

        // check binary serialization
        {
        }
    }

    // check vector function serialization
    for(int i = 1; i<=10; ++i)
    {
        typedef learn::linear_function<float, learn::generic_multi_function<float> >    linear_function_type;
        typedef learn::vector_function<float, linear_function_type>                     vector_function_type;

        linear_function_type linFunc;
        {
            linFunc.set_out_min(-1.0f);
            linFunc.set_out_max(1.0f);
            linFunc.set_regularization(0.01f);

            linFunc.push_function( learn::make_multi_generic( learn::constant<float>(1.0f) ) );
            for (int j = 0; j<i; ++j)
            {
                linFunc.push_function( learn::make_multi_generic( learn::select<float>(j) ) );
                for (int k = 0; k<i; ++k)
                {
                    linFunc.push_function( learn::make_multi_generic( learn::select_product<float>(j, k) ) );
                }
            }
        }

        vector_function_type vecFunc[2];
        vecFunc[0] = vector_function_type(i, linFunc);

        // check text serialization
        {
            std::ostringstream oss;
            boost::archive::text_oarchive oa(oss);
            oa << vecFunc[0];

            std::istringstream iss( oss.str() );
            boost::archive::text_iarchive ia(iss);
            ia >> vecFunc[1];

            BOOST_REQUIRE( compare( vecFunc[0], vecFunc[1], std::numeric_limits<float>::min() ) );
        }

        // check binary serialization
        {
        }
    }
}

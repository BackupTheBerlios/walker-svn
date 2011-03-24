#ifndef __WALKER_YARD_SERIALIZATION_HPP__
#define __WALKER_YARD_SERIALIZATION_HPP__

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>

namespace learn {

// forward
template<typename Function> class function_with_derivative;
template<typename Function> class derivative;
template<typename Function> class abstract_functor;
template<typename Function> class abstract_multi_functor;

template<typename T> class linear;
template<typename T> class logistic;
template<typename T> class hyperbolic_tangent;
template<typename T> class generic_function;

template<typename T> class select;
template<typename T> class select_anti_quad;
template<typename T> class constant;
template<typename T> class sum;
template<typename T> class gaussian;
template<typename T> class select_product;
template<typename T> class select_sum;
template<typename T> class select_subtract;
template<typename T> class generic_multi_function;
template<typename T> class abstract_multi_function;
template<typename T> class select_sum3;

template<typename Result, typename Argument>
class abstract_function;

template<typename ValueType, typename ScalarFunction>
class vector_function;

template<typename ValueType, typename FunctionType>
class linear_function;

template<typename ValueType, typename ActivateFunction>
class neural_network;

template<typename NeuralNetwork>
class scalar_neural_network;

namespace io {

template<typename Object>
void deserialize(const char* filename, Object& obj)
{
    std::ifstream ifs(filename);
    boost::archive::text_iarchive ia(ifs);
    ia >> obj;
}

template<typename Object>
void serialize(const char* filename, const Object& obj)
{
    std::ofstream ofs(filename);
    boost::archive::text_oarchive oa(ofs);
    oa << obj;
}

class access
{
public:
    template<typename Archive,
             typename ValueType,
             typename ActivateFunction>
    static void save(Archive&                                              ar, 
                     const neural_network<ValueType, ActivateFunction>&    nn, 
                     unsigned int                                          version)
    {
        typedef neural_network<ValueType, ActivateFunction> nn_type;

        ar & nn.numLayers;
        ar & nn.numNeurons;
        ar & nn.numLinks;
        ar & nn.useIndent;
        
        for (size_t i = 0; i<nn.numLayers; ++i) 
	    {
            int first = std::distance( nn.first_neuron(), (typename nn_type::neuron_const_iterator)nn.layers[i].firstNeuron );
            int last  = std::distance( nn.first_neuron(), (typename nn_type::neuron_const_iterator)nn.layers[i].endNeuron );
            ar & first & last;
	    }

	    for (size_t i = 0; i<nn.numNeurons; ++i) 
	    {
            int first = std::distance( nn.first_link(), (typename nn_type::link_const_iterator)nn.neurons[i].firstLink );
            int last  = std::distance( nn.first_link(), (typename nn_type::link_const_iterator)nn.neurons[i].endLink );

            ar & nn.neurons[i].input;
            ar & nn.neurons[i].output;
            ar & nn.neurons[i].gradient;
            ar & nn.neurons[i].errorGradient;
            ar & nn.neurons[i].activateFunc;
            ar & first & last;
	    }

	    for (size_t i = 0; i<nn.numLinks; ++i) 
	    {
            int source = std::distance( nn.first_neuron(), (typename nn_type::neuron_const_iterator)nn.links[i].source );
            int target = std::distance( nn.first_neuron(), (typename nn_type::neuron_const_iterator)nn.links[i].target );
            ar & nn.links[i].weight & source & target;
	    }
    }

    template<typename Archive,
             typename ValueType,
             typename ActivateFunction>
    static void load(Archive&                                          ar, 
                     neural_network<ValueType, ActivateFunction>&      nn, 
                     unsigned int                                      version)
    {
        typedef neural_network<ValueType, ActivateFunction> nn_type;

        ar & nn.numLayers;
        ar & nn.numNeurons;
        ar & nn.numLinks;
        ar & nn.useIndent;

        nn.layers.reset(new typename nn_type::layer[nn.numLayers]);
        nn.neurons.reset(new typename nn_type::neuron[nn.numNeurons]);
        nn.links.reset(new typename nn_type::link[nn.numLinks]);

        int first, end;
        for (size_t i = 0; i<nn.numLayers; ++i) 
	    {
            ar & first & end;
		    nn.layers[i].firstNeuron = nn.first_neuron() + first;
		    nn.layers[i].endNeuron   = nn.first_neuron() + end;
	    }

	    for (size_t i = 0; i<nn.numNeurons; ++i) 
	    {
            ar & nn.neurons[i].input;
            ar & nn.neurons[i].output;
            ar & nn.neurons[i].gradient;
            ar & nn.neurons[i].errorGradient;
            ar & nn.neurons[i].activateFunc;
            ar & first & end;

		    nn.neurons[i].firstLink = nn.first_link() + first;
		    nn.neurons[i].endLink   = nn.first_link() + end;
	    }

	    for (size_t i = 0; i<nn.numLinks; ++i) 
	    {
            ar & nn.links[i].weight & first & end;
            nn.links[i].source = nn.first_neuron() + first;
		    nn.links[i].target = nn.first_neuron() + end;
	    }
    }

    template<typename Archive,
             typename NeuralNetwork>
    static void serialize(Archive&                              ar, 
                          scalar_neural_network<NeuralNetwork>& nn,
                          unsigned int                          version)
    {
        ar & nn.network;
    }

    template<typename Archive,
             typename ValueType,
             typename FunctionType>
    static void serialize(Archive&                                  ar, 
                          linear_function<ValueType, FunctionType>& linFunc,
                          unsigned int                              version)
    {
        ar & linFunc.outMin;
        ar & linFunc.outMax;
        ar & linFunc.tau;

        ar & linFunc.params;
        ar & linFunc.functions;
    }

    template<typename Archive,
             typename ValueType,
             typename ScalarFunction>
    static void serialize(Archive&                                           ar,
                          learn::vector_function<ValueType, ScalarFunction>& vectorFunc,
                          unsigned int                                       version)
    {
        ar & vectorFunc.numParams;
        ar & vectorFunc.components;
    }
};

} // namespace io
} // namespace learn

namespace boost {
namespace serialization {

template<typename Archive,
         typename ValueType,
         typename ActivateFunction>
void save(Archive&                                                  ar, 
          const learn::neural_network<ValueType, ActivateFunction>& nn, 
          unsigned int                                              version)
{
    learn::io::access::save(ar, nn, version);
}

template<typename Archive,
         typename ValueType,
         typename ActivateFunction>
void load(Archive&                                             ar, 
          learn::neural_network<ValueType, ActivateFunction>&  nn, 
          unsigned int                                         version)
{
    learn::io::access::load(ar, nn, version);
}

template<typename Archive,
         typename ValueType,
         typename ActivateFunction>
inline void serialize(Archive&                                             ar,
                      learn::neural_network<ValueType, ActivateFunction>&  nn,
                      unsigned int                                         version)
{
    boost::serialization::split_free(ar, nn, version); 
}

template<typename Archive,
         typename NeuralNetwork>
void serialize(Archive&                                     ar, 
               learn::scalar_neural_network<NeuralNetwork>& nn,
               unsigned int                                 version)
{
    learn::io::access::serialize(ar, nn, version);
}

template<typename Archive,
         typename ValueType,
         typename FunctionType>
inline void serialize(Archive&                                          ar,
                      learn::linear_function<ValueType, FunctionType>&  linFunc,
                      unsigned int                                      version)
{
    learn::io::access::serialize(ar, linFunc, version);
}

template<typename Archive,
         typename ValueType,
         typename ScalarFunction>
inline void serialize(Archive&                                              ar,
                      learn::vector_function<ValueType, ScalarFunction>&    vectorFunc,
                      unsigned int                                          version)
{
    learn::io::access::serialize(ar, vectorFunc, version);
}

template<typename ValueType, typename Archive>
void register_function_types(Archive& ar)
{
    ar.template register_type< learn::abstract_functor< learn::linear<ValueType> > >();
    ar.template register_type< learn::abstract_functor< learn::logistic<ValueType> > >();
    ar.template register_type< learn::abstract_functor< learn::hyperbolic_tangent<ValueType> > >();

    ar.template register_type< learn::abstract_functor< learn::derivative< learn::linear<ValueType> > > >();
    ar.template register_type< learn::abstract_functor< learn::derivative< learn::logistic<ValueType> > > >();
    ar.template register_type< learn::abstract_functor< learn::derivative< learn::hyperbolic_tangent<ValueType> > > >();

    ar.template register_type< learn::abstract_functor< learn::function_with_derivative< learn::linear<ValueType> > > >();
    ar.template register_type< learn::abstract_functor< learn::function_with_derivative< learn::logistic<ValueType> > > >();
    ar.template register_type< learn::abstract_functor< learn::function_with_derivative< learn::hyperbolic_tangent<ValueType> > > >();
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                             ar, 
               learn::generic_function<ValueType>&  func, 
               unsigned int                         version)
{
    register_function_types<ValueType>(ar);
    ar & func.func;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                 ar, 
               learn::derivative< learn::generic_function<ValueType> >& func, 
               unsigned int                                             version)
{
    register_function_types<ValueType>(ar);
    ar & func.func;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                                 ar, 
               learn::function_with_derivative< learn::generic_function<ValueType> >&   func, 
               unsigned int                                                             version)
{
    register_function_types<ValueType>(ar);
    ar & func.func;
}

template<typename Archive,
         typename Functor>
void serialize(Archive&                          ar, 
               learn::abstract_functor<Functor>& func, 
               unsigned int                      /*version*/)
{
    void_cast_register( static_cast<learn::abstract_functor<Functor>*>(0),
                        static_cast<learn::abstract_function<typename Functor::argument_type, typename Functor::result_type>*>(0) );

    ar & func.func;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                     /*ar*/, 
               learn::linear<ValueType>&    /*func*/, 
               unsigned int                 /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                         /*ar*/, 
               learn::logistic<ValueType>&      /*func*/, 
               unsigned int                     /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                 /*ar*/, 
               learn::hyperbolic_tangent<ValueType>&    /*func*/, 
               unsigned int                             /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                       /*ar*/, 
               learn::derivative< learn::linear<ValueType> >& /*func*/, 
               unsigned int                                   /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                           /*ar*/, 
               learn::derivative< learn::logistic<ValueType> >&   /*func*/, 
               unsigned int                                       /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                     /*ar*/, 
               learn::derivative< learn::hyperbolic_tangent<ValueType> >&   /*func*/, 
               unsigned int                                                 /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                     /*ar*/, 
               learn::function_with_derivative< learn::linear<ValueType> >& /*func*/, 
               unsigned int                                                 /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                         /*ar*/, 
               learn::function_with_derivative< learn::logistic<ValueType> >&   /*func*/, 
               unsigned int                                                     /*version*/)
{
    /* nothing to serialize */
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                                                 /*ar*/, 
               learn::function_with_derivative< learn::hyperbolic_tangent<ValueType> >& /*func*/, 
               unsigned int                                                             /*version*/)
{
    /* nothing to serialize */
}

template<typename ValueType, typename Archive>
void register_multi_function_types(Archive& ar)
{
    ar.template register_type< learn::abstract_multi_functor< learn::select<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::select_anti_quad<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::constant<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::sum<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::select_product<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::select_sum<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::select_sum3<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::select_subtract<ValueType> > >();
    ar.template register_type< learn::abstract_multi_functor< learn::gaussian<ValueType> > >();
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                                  ar, 
               learn::generic_multi_function<ValueType>& func, 
               unsigned int                              version)
{
    register_multi_function_types<ValueType>(ar);
    ar & func.func;
}

template<typename Archive,
         typename Functor>
void serialize(Archive&                                 ar, 
               learn::abstract_multi_functor<Functor>&  func, 
               unsigned int                             /*version*/)
{
    void_cast_register( static_cast<learn::abstract_multi_functor<Functor>*>(0),
                        static_cast<learn::abstract_multi_function<typename Functor::value_type>*>(0) );

    ar & func.func;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                     ar, 
               learn::select<ValueType>&    func, 
               unsigned int                 /*version*/)
{
    ar & func.index;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                     ar, 
               learn::select_anti_quad<ValueType>&    func, 
               unsigned int                 /*version*/)
{
    ar & func.index;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                     ar, 
               learn::constant<ValueType>&  func, 
               unsigned int                 /*version*/)
{
    ar & func.val;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                     /*ar*/, 
               learn::sum<ValueType>&       /*func*/, 
               unsigned int                 /*version*/)
{
    /* nothing to serialize */
}


template<typename Archive,
         typename ValueType>
void serialize(Archive&                          ar, 
               learn::select_product<ValueType>& func, 
               unsigned int                      /*version*/)
{
    ar & func.first;
    ar & func.second;
}
template<typename Archive,
         typename ValueType>
void serialize(Archive&                          ar, 
               learn::select_sum<ValueType>& func, 
               unsigned int                      /*version*/)
{
    ar & func.first;
    ar & func.second;
}
template<typename Archive,
         typename ValueType>
void serialize(Archive&                          ar, 
               learn::select_subtract<ValueType>& func, 
               unsigned int                      /*version*/)
{
    ar & func.first;
    ar & func.second;
}
template<typename Archive,
         typename ValueType>
void serialize(Archive&                    ar, 
               learn::gaussian<ValueType>& func, 
               unsigned int                /*version*/)
{
    ar & func.mean;
    ar & func.sigma;
}

template<typename Archive,
         typename ValueType>
void serialize(Archive&                          ar, 
               learn::select_sum3<ValueType>& func, 
               unsigned int                      /*version*/)
{
    ar & func.first;
    ar & func.second;
    ar & func.third;
}

/*
template< typename T, typename Container >
inline std::ostream& operator << (std::ostream& out, const learn::matrix<T, Container> & mat)
{
    out << mat.height << ' ' << mat.width << std::endl;
    for (size_t i = 0; i < mat.height*mat.width; ++i)
    {
        out << mat.values[i] << ' ';
    }
    out << std::endl;
    return out;
}

template < typename T, typename ParamsContainer, typename SampleContainer >
inline std::ostream& operator << (std::ostream& out, 
                                  const learn::radial_basis_function<T, ParamsContainer, SampleContainer>& func)
{
    out << func.means;

    out << func.radius << ' ';
    out << func.tau << ' ';
    out << func.outMin << ' ' << func.outMax << ' ';
    out << func.dimensions << ' ' << func.numParams << std::endl;
    for (size_t i = 0; i < func.numParams; ++i)
    {
        out << func.params[i] << ' ';
    }
    out << std::endl;
    return out;
}

template < typename ScalarFunction >
inline std::ostream& operator << (std::ostream& out,
                                  const learn::vector_function<ScalarFunction>& func)
{
    out << func.components.size() << std::endl;
    for (size_t i = 0; i < func.components.size(); ++i)
    {
        out << func.components[i];
    }
    return out;
}


template< typename T,
          typename Container >
inline std::istream& operator >> (std::istream& input,
                                  learn::matrix<T, Container> & mat)
{
    input >> mat.height >> mat.width;
    mat.values = Container(mat.height*mat.width);
    for (size_t i = 0; i < mat.height*mat.width; ++i)
    {
        input >> mat.values[i];
    }
    return input;
}

template < typename T,
           typename ParamsContainer,
           typename SampleContainer >
inline std::istream& operator >> (std::istream& input,
                                  learn::radial_basis_function<T, ParamsContainer, SampleContainer>& func)
{
    input >> func.means;

    input >> func.radius;
    input >> func.tau;
    input >> func.outMin >> func.outMax;
    input >> func.dimensions >> func.numParams;
    func.params =  ParamsContainer(func.numParams);
    for (size_t i = 0; i < func.numParams; ++i)
    {
        input >> func.params[i];
    }
    return input;
}

template < typename ScalarFunction >
inline std::istream& operator >> (std::istream& input,
                                  learn::vector_function<ScalarFunction>& func)
{
    int size;
    input >> size;
    func.components = std::vector<ScalarFunction>(size);
    for (size_t i = 0; i < func.components.size(); ++i)
    {
        input >> func.components[i];
    }
    return input;
}
*/
} // namespace serialization
} // namespace boost

#endif // __WALKER_YARD_SERIALIZATION_HPP__

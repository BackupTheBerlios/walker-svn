#ifndef __WALKER_YARD_VECTOR_FUNCTION_HPP__
#define __WALKER_YARD_VECTOR_FUNCTION_HPP__

#include <vector>

namespace learn {

// forward
namespace io {
    class access;
}

template<typename ValueType,
         typename ScalarFunction>
class vector_function
{
friend class io::access;
public:
    typedef ScalarFunction  scalar_function_type;
    typedef ValueType       value_type;

    typedef std::vector<scalar_function_type>           function_vector;
    typedef typename function_vector::const_iterator    function_const_iterator;

public:
    vector_function() :
        numParams(0)
    {}

    vector_function(size_t                      numOutputs, 
                    const scalar_function_type& function) :
        components(numOutputs, function)
    {
        dirty_components();
    }

    template<typename Iterator>
    vector_function(Iterator firstFunction, Iterator endFunction) :
        components(firstFunction, endFunction)
    {
        dirty_components();
    }

    size_t num_components() const   { return components.size(); }
    size_t num_inputs() const       { return components[0].num_inputs(); }
    size_t num_outputs() const      { return components.size(); }
    size_t num_params() const       { return numParams; }

    function_const_iterator first_function() const   { return components.begin(); }
    function_const_iterator end_function() const     { return components.end(); }

    template<typename OutIterator>
    void get_params(OutIterator outParam)
    {
        //throw std::exception("not implemented");
        for (size_t i = 0; i < num_components(); ++i)
        {
            components[i].get_params(outParam);
            outParam += numParams/num_components();
        }
       // std::copy(params.begin(), params.end(), outParam);
    }

    template<typename InIterator>
    void set_params(InIterator beginParam, InIterator endParam)
    {
        //throw std::exception("not implemented");
        for (size_t i = 0; i < num_components(); ++i)
        {
            components[i].set_params(beginParam, beginParam + numParams/num_components());
            beginParam += numParams/num_components();
        }
        assert(beginParam == endParam);
        
        // std::copy( beginParam, endParam, params.begin() );
    }

    template<typename InIterator, typename OutIterator>
    void get_gradient(InIterator beginInput, InIterator endInput, OutIterator outParam)
    {
        //throw std::exception("not implemented");
        for (size_t i = 0; i < num_components(); ++i)
        {
            components[i].get_gradient(beginInput, endInput, outParam);
            outParam += numParams/num_components();
        }
       // std::copy(params.begin(), params.end(), outParam);
    }
    
    template<typename InIterator, typename OutIterator>
    void compute(InIterator beginInput, InIterator endInput, OutIterator out) const
    {
        for (size_t i = 0; i<components.size(); ++i) {
            *out++ = components[i].compute(beginInput, endInput);
        }
    }

    template<typename InIterator1, typename InIterator2>
    void update( InIterator1    beginInput, 
                 InIterator1    endInput, 
                 InIterator2    beginNewOut, 
                 InIterator2    endNewOut, 
                 value_type     stepSize )
    {
        for (size_t i = 0; i<components.size(); ++i, ++beginNewOut)
        {
            components[i].update(beginInput, endInput, *beginNewOut, stepSize);
        }
        assert(beginNewOut == endNewOut);
    }

private:
    void dirty_components()
    {
        numParams = 0;
        for (size_t i = 0; i<components.size(); ++i)
        {
            assert( components[i].num_inputs()  == components[0].num_inputs() );
            numParams += components[i].num_params();
        }
    }

public:
    size_t          numParams;
    function_vector components;
};

} // namespace learn

#endif // __WALKER_YARD_VECTOR_FUNCTION_HPP__
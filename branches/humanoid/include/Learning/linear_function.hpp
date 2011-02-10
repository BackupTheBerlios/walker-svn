#ifndef __WALKER_YARD_LEARNING_LINEAR_FUNCTION_HPP__
#define __WALKER_YARD_LEARNING_LINEAR_FUNCTION_HPP__

#include "function.hpp"

namespace learn {

// forward
namespace io {
    class access;
}

template <typename ValueType,
          typename Function>
class linear_function
{
friend class io::access;
public:
    typedef ValueType                       value_type;
    typedef Function                        function_type;

    typedef std::vector<value_type>                 params_array;
    typedef typename params_array::iterator         parameter_iterator;
    typedef typename params_array::const_iterator   parameter_const_iterator;

    typedef std::vector<function_type>              function_array;
    typedef typename function_array::iterator       function_iterator;
    typedef typename function_array::const_iterator function_const_iterator;

public:
    linear_function() {}
    linear_function(value_type   _outMin,
                    value_type   _outMax) :
        outMin(_outMin),
        outMax(_outMax),
        tau(0.0)
    {
    }

    void push_function(function_type function)
    {
        functions.push_back(function);
        params.push_back(value_type());
    }

    void pop_function()
    {
        functions.pop_back();
        params.pop_back();
    }

    parameter_iterator first_parameter()  { return params.begin(); }
    parameter_iterator end_parameter()    { return params.end(); }

    function_iterator first_function()  { return functions.begin(); }
    function_iterator end_function()    { return functions.end(); }

    function_const_iterator first_function() const  { return functions.begin(); }
    function_const_iterator end_function() const    { return functions.end(); }

    parameter_const_iterator first_parameter() const { return params.begin(); }
    parameter_const_iterator end_parameter() const   { return params.end(); }

    size_t num_inputs() const   { return params.size(); }
    size_t num_params() const   { return params.size(); }

    void set_out_min(value_type outMin_)        { outMin = outMin_; }
    void set_out_max(value_type outMax_)        { outMax = outMax_; }
    void set_regularization(value_type tau_)    { tau = tau_; }

    value_type out_min() const          { return outMin; }
    value_type out_max() const          { return outMax; }
    value_type regularization() const   { return tau; }

    template<typename InIterator>
    value_type compute(InIterator beginInput, InIterator endInput) const
    {
        value_type result = approximate(beginInput, endInput);
        result = (result < outMin)? outMin: (result > outMax)? outMax: result;
        return result;
    }

    template<typename InIterator, typename OutIterator>
    void get_gradient(InIterator    beginInput, 
                      InIterator    endInput, 
                      OutIterator   outGrad) const
    {
        for (size_t i = 0; i < params.size(); ++i, ++outGrad)
        {
            *outGrad = functions[i](beginInput, endInput);
        }
    }

    template<typename OutIterator>
    void get_params(OutIterator outParam) const
    {
        std::copy(params.begin(), params.end(), outParam);
    }

    template<typename InIterator>
    void set_params(InIterator beginParam, 
                    InIterator endParam)
    {
        std::copy( beginParam, endParam, params.begin() );
    }

    template<typename InIterator>
    void update(InIterator beginInput,
                InIterator endInput,
                value_type newOut,
                value_type stepSize)
    {
		newOut = (newOut > outMax)? outMax : (newOut < outMin)? outMin : newOut;
        value_type out = approximate(beginInput, endInput);
        for (size_t i = 0; i < params.size(); ++i)
        {
            params[i] *= 1 - tau;
            params[i] += stepSize*(newOut - out)*functions[i](beginInput, endInput);
        }
    }

private:
    template<typename InIterator>
    value_type approximate(InIterator beginInput, InIterator endInput) const
    {
        value_type result = 0;
        for (size_t i = 0; i < params.size(); ++i)
        {
            result += params[i]*functions[i](beginInput, endInput);
        }
        return result;
    }

private:
    value_type      outMin;
    value_type      outMax;
    value_type      tau;

    params_array    params;
    function_array  functions;
};

template<typename T>
linear_function<T, generic_multi_function<T> > make_linear_rbf(size_t   numInputs, 
                                                               T        outMin, 
                                                               T        outMax)
{
    linear_function<T, generic_multi_function<T> > linear(outMin, outMax);

    std::vector<T> mean1(numInputs, -1.0);
    std::vector<T> mean2(numInputs, 1.0);
    std::vector<T> mean3(numInputs, 0.0);
    std::vector<T> mean4(numInputs, -1.0);
    std::vector<T> mean5(numInputs, 1.0);
    mean4[0] = 1.0; //mean4[1] = 1.0;
    mean5[0] = -1.0;// mean5[1] = -1.0;

    T sigma = 1.0;

    //linear.addFunction(make_function(new gauss<T>(mean1.begin(), mean1.end(), sigma)));
    //linear.addFunction(make_function(new gauss<T>(mean2.begin(), mean2.end(), sigma)));
    //linear.addFunction(make_function(new gauss<T>(mean3.begin(), mean3.end(), sigma)));
    //linear.addFunction(make_function(new gauss<T>(mean4.begin(), mean4.end(), sigma)));
    //linear.addFunction(make_function(new gauss<T>(mean5.begin(), mean5.end(), sigma)));

    //linear.push_function( make_multi_generic( constant<T>(1.0) ) );
    linear.push_function( make_multi_generic( select_anti_quad<T>(0) ) );
    linear.push_function( make_multi_generic( select_anti_quad<T>(1) ) );
   // linear.push_function( make_multi_generic( select<T>(2) ) );
   // linear.push_function( make_multi_generic( select<T>(3) ) );
   // linear.push_function( make_multi_generic( select<T>(6) ) );
    //linear.push_function( make_multi_generic( select<T>(7) ) );

    linear.push_function( make_multi_generic( select_sum<T>(0, 2) ) );
    linear.push_function( make_multi_generic( select_sum<T>(1, 3) ) );
//    linear.push_function( make_multi_generic( select_sum3<T>(0, 2, 4) ) );
//    linear.push_function( make_multi_generic( select_sum3<T>(1, 3, 5) ) );
    //linear.push_function( make_multi_generic( select_sum<T>(2, 6) ) );
    //linear.push_function( make_multi_generic( select_sum<T>(3, 7) ) );
    //linear.push_function( make_multi_generic( select_sum<T>(0, 2) ) );
    //linear.push_function( make_multi_generic( select_sum<T>(1, 3) ) );
    //linear.push_function( make_multi_generic( select<T>(4) ) );
    //linear.push_function( make_multi_generic( select<T>(5) ) );
    //linear.push_function( make_multi_generic( select<T>(6) ) );
    //linear.push_function( make_multi_generic( select<T>(7) ) );
    
    //linear.push_function( make_multi_generic( select_subtract<T>(2, 6) ) );
    //linear.push_function( make_multi_generic( select_subtract<T>(3, 7) ) );
    //linear.push_function( make_multi_generic( select_product<T>(0, 1) ) );
    //linear.push_function( make_multi_generic( select_product<T>(0, 0) ) );
    //linear.push_function( make_multi_generic( select_product<T>(1, 1) ) );
    //linear.push_function( make_multi_generic( select_product<T>(2, 3) ) );
    //linear.push_function( make_multi_generic( select_product<T>(2, 2) ) );
   // linear.push_function( make_multi_generic( select_product<T>(3, 3) ) );
    //linear.push_function( make_multi_generic( select_product<T>(1, 2) ) );
    //linear.push_function( make_multi_generic( select_product<T>(1, 3) ) );
   // linear.push_function( make_multi_generic( select_product<T>(0, 2) ) );
   // linear.push_function( make_multi_generic( select_product<T>(0, 3) ) );

    //linear.addFunction(make_function(new sum<T>()));

    //linear.addFunction(make_function(new tanh_sum<T>()));
    return linear;
}

} // namespace learn

#endif  // __WALKER_YARD_LEARNING_LINEAR_FUNCTION_HPP__

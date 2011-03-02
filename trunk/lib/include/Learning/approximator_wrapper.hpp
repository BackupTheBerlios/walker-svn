#ifndef __WALKER_YARD_RADIAL_BASIS_FUNCTION_HPP__
#define __WALKER_YARD_RADIAL_BASIS_FUNCTION_HPP__

#include <vector>
#include <cassert>
#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>

//#include "matrix.hpp"

namespace learn {
    
namespace ublas = boost::numeric::ublas;

template < typename T,
           typename ParamsContainer = std::vector<T>,
           typename SampleContainer = std::vector<T> >
class radial_basis_function
{

public:
    typedef T                               value_type;
	typedef ublas::matrix<value_type>		sample_type;
	typedef boost::shared_ptr<sample_type>  sample_ptr;

    typedef ParamsContainer                 params_array;
    typedef SampleContainer                 sample_array;
    typedef ublas::matrix<value_type>       sample_matrix;

public:
    radial_basis_function(){}
    template<typename Iterator>
    radial_basis_function( size_t       _dimensions,
                           size_t       _sampleSize,
                           T            _radius,
                           T            _outMin,
                           T            _outMax,
                           Iterator     _firstMean,
                           Iterator     _endMean ):
                means(_sampleSize, _dimensions),
                radius(_radius),
                dimensions(_dimensions),
                numParams(_sampleSize),
                params(_sampleSize),
                outMin(_outMin),
                outMax(_outMax),
                tau(0.0)
    {
        std::copy( _firstMean, _endMean, means.data().begin() );
    }

    radial_basis_function( size_t               _dimensions,
                           size_t               _sampleSize,
                           T                    _radius,
                           T                    _outMin,
                           T                    _outMax,
                           const sample_array&  _means ):
                means(_sampleSize, _dimensions),
                radius(_radius),
                dimensions(_dimensions),
                numParams(_sampleSize),
                params(_sampleSize),
                outMin(_outMin),
                outMax(_outMax),
                tau(0.0)
    {
        std::copy( _means.begin(), _means.end(), means.data().begin() );
    }

    radial_basis_function( size_t               _dimensions,
                           size_t               _sampleSize,
                           T                    _radius,
                           T                    _outMin,
                           T                    _outMax,
                           const sample_matrix& _means ):
                means(_means),
                radius(_radius),
                dimensions(_dimensions),
                numParams(_sampleSize),
                params(_sampleSize),
                outMin(_outMin),
                outMax(_outMax),
                tau(0.0)
    {
    }

    /*
    template <typename MatrixType>
    radial_basis_function( size_t       _dimensions,
                           size_t       _sampleSize,
                           T            _radius,
                           T            _outMin,
                           T            _outMax,
                           MatrixType&  _sample) :
        dimensions(_dimensions),
        params(_sampleSize),
        numParams(_sampleSize),
        radius(_radius),
        tau(0.00),
        outMin(_outMin),
        outMax(_outMax),
		means(dimensions, _sampleSize, _sample)
    {
    }

    template <typename MatrixType>
    radial_basis_function( size_t       _dimensions,
                           size_t       _sampleSize,
                           T            _radius,
                           T            _outMin,
                           T            _outMax,
                           MatrixType&  sample) :
        dimensions(_dimensions),
        params(_sampleSize),
        radius(_radius),
        tau(0.00),
        outMin(_outMin),
        outMax(_outMax),
		means(sample_ptr(new sample_type()))
    {
        for (size_t i = 0; i < _sampleSize; ++i)
        {
            means->push_back(std::vector<T>(dimensions));
            for (size_t j = 0; j < dimensions; ++j)
            {
                (*means)[i][j] = sample[i][j];
            }
        }
    }

    radial_basis_function( T            _radius,
                           T            _outMin,
                           T            _outMax,
                           sample_ptr  sample) :
        params(sample->size()),
        radius(_radius),
        tau(0.00),
        outMin(_outMin),
        outMax(_outMax),
		means(sample)
    {
		dimensions = sample->begin()->size();
    }*/

    size_t num_inputs() const   { return dimensions; }
    size_t num_params() const   { return params.size(); }

    template<typename InIterator>
    T compute(InIterator beginInput, InIterator endInput) const
    {
        T result = approximate(beginInput, endInput);
        result = (result < outMin)? outMin: (result > outMax)? outMax: result;
        return result;
    }

    template<typename InIterator, typename OutIterator>
    void get_gradient(InIterator beginInput, InIterator endInput, OutIterator outGrad) const
    {
        for (size_t i = 0; i < params.size(); ++i, ++outGrad)
        {
            *outGrad = Phi(beginInput, endInput, i);
        }
    }

    template<typename OutIterator>
    void get_params(OutIterator outParam) const
    {
        std::copy(params.begin(), params.end(), outParam);
    }

    template<typename InIterator>
    void set_params(InIterator beginParam, InIterator endParam)
    {
        std::copy( beginParam, endParam, params.begin() );
    }

    template<typename InIterator1>
    void update( InIterator1 beginInput,
                 InIterator1 endInput,
                 T           newOut,
                 T           stepSize )
    {
		newOut = (newOut > outMax)? outMax : (newOut < outMin)? outMin : newOut;
        T out = approximate(beginInput, endInput);
        for (size_t i = 0; i < params.size(); ++i)
        {
            params[i] *= 1 - tau;
            params[i] += stepSize*(newOut - out)*Phi(beginInput, endInput, i);
        }
		out = approximate(beginInput, endInput);
    }

private:
    template<typename InIterator>
    T approximate(InIterator beginInput, InIterator endInput) const
    {
        T result = 0;
        for (size_t i = 0; i < params.size(); ++i)
        {
            result += params[i]*Phi(beginInput, endInput, i);
        }
        return result;
    }
    template<typename InIterator>
    T Phi(InIterator beginInput, InIterator endInput, int meanIndex) const
    {
        T distance = 0;
        for (size_t i = 0; i < dimensions; ++i, ++beginInput)
        {
            distance += (*beginInput - means(meanIndex,i)) * (*beginInput - means(meanIndex,i));
        }

        assert(beginInput == endInput);
        return exp(-distance/(2*radius*radius));
    }

public:
    T outMin;
    T outMax;
    size_t dimensions;
    size_t numParams;
    T radius;
    T tau;
    sample_matrix   means;
    params_array    params;
};

} // namespace learn

#endif  // __WALKER_YARD_RADIAL_BASIS_FUNCTION_HPP__

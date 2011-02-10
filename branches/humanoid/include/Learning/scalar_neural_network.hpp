#ifndef __WALKER_YARD_LEARNING_SCALAR_NEURAL_NETWORK_HPP__
#define __WALKER_YARD_LEARNING_SCALAR_NEURAL_NETWORK_HPP__

#include "neural_network.hpp"

namespace learn {

template<typename NetworkType>
class scalar_neural_network
{
friend class io::access;
public:
    typedef NetworkType                         network_type;
    typedef typename network_type::value_type   value_type;

public:
    scalar_neural_network() {}
    scalar_neural_network(network_type network_) :
        network(network_)
    {
        assert(network.num_outputs() == 1);
    }

    template<typename InIterator>
    value_type compute(InIterator     beginInput,
                       InIterator     endInput) const
    {
        assert(network.num_outputs() == 1);

        value_type value;
        network.compute(beginInput, endInput, &value);
        return value - value_type(1.0);
    }

    template<typename InIterator1>
    void update( InIterator1    beginInput,
                 InIterator1    endInput,
                 value_type     output,
                 value_type     stepSize )
    {
        assert(network.num_outputs() == 1);
        output += value_type(1.0);
        network.update(beginInput, endInput, &output, &output + 1, stepSize);
    }

    template<typename InIterator>
    void set_params(InIterator beginParam, 
                    InIterator endParam)
    {
        network.set_params(beginParam, endParam);
    }

    template<typename OutIterator>
    void get_params(OutIterator outParam)
    {
        network.get_params(outParam);
    }

    template<typename InIterator, typename OutIterator>
    void get_gradient(InIterator    beginInput, 
                      InIterator    endInput, 
                      OutIterator   outGrad)
    {
        network.get_gradient(beginInput, endInput, outGrad);
    }

    int num_params()
    {
        return network.num_params();
    }

    size_t num_inputs() const   
    { 
        return network.num_inputs(); 
    }
private:
    network_type network;
};

} // namespace learn

#endif // __WALKER_YARD_LEARNING_SCALAR_NEURAL_NETWORK_HPP__
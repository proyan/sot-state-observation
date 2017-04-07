#include <sstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-state-observation/filter.hh>

using namespace std;

namespace sotStateObservation
{
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( Filter, "Filter" );

    Filter::Filter( const std::string & inName):
        Entity(inName),
        inputSIN_ (NULL, "Filter("+inName+")::input(vector)::sin"),
        outputSOUT_ (NULL, "Filter("+inName+")::output(vector)::sout"),
        on_(false), time_(0), n_(10), distr_(10), filter_(0)
    {

        signalRegistration (inputSIN_ << outputSOUT_);

        std::string docstring;

        docstring  =
                "\n"
                "    Enable (true) or disable (false) the filter \n"
                "\n";

        addCommand(std::string("setOn"),
             new
             ::dynamicgraph::command::Setter <Filter,bool>
                (*this, &Filter::setOn, docstring));

        docstring  =
                "\n"
                "    Set the wondow size \n"
                "\n";

        addCommand(std::string("setWindowSize"),
             new
             ::dynamicgraph::command::Setter <Filter,unsigned>
                (*this, &Filter::setWindowSize, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

        docstring  =
                "\n"
                "    Set the filter type (0 by default)\n"
                "    \t 0: Gaussian filter \n"
                "    \t 1: Average filter \n"
                "\n";

        addCommand(std::string("setFilter"),
             new
             ::dynamicgraph::command::Setter <Filter,unsigned>
                (*this, &Filter::setFilter, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

        docstring  =
                "\n"
                "    Get the wondow size \n"
                "\n";

        addCommand(std::string("getWindowSize"),
             new
             ::dynamicgraph::command::Getter <Filter,unsigned>
                (*this, &Filter::getWindowSize, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

        docstring  =
                "\n"
                "    Get the filter type (0 by default)\n"
                "    \t 0: Gaussian filter \n"
                "    \t 1: Average filter \n"
                "\n";

        addCommand(std::string("getFilter"),
             new
             ::dynamicgraph::command::Getter <Filter,unsigned>
                (*this, &Filter::getFilter, docstring));

        outputSOUT_.setFunction(boost::bind(&Filter::getOutput, this, _1, _2));

        updateDistribution();

    }

    Filter::~Filter()
    {
    }

    stateObservation::Vector Filter::averageDistribution(const unsigned n)
    {
        vec_.resize(n);
        vec_.setOnes();
        vec_=double(1./double(n))*vec_;
        return vec_;
    }

    stateObservation::Vector Filter::gaussianDistribution(const unsigned n, const double mean, const double stddev)
    {
        vec_.resize(n);
        double sum=0.;
        for(int i=0; i<vec_.size();++i)
        {
            vec_[i]=(1./(stddev*std::sqrt(6.28)))*std::exp(-0.5*((i-mean)/stddev)*((i-mean)/stddev));
            sum+=vec_[i];
        }
        vec_=(1./sum)*vec_;
        return vec_;
    }

    stateObservation::Vector Filter::updateGaussianDistribution(const unsigned n)
    {
        double mean=0;
        double stddev=std::sqrt((n-mean)*(n-mean)/4.6); // the first element of the window correspond to 10% of the last element.
        return gaussianDistribution(n,mean,stddev);
    }

    void Filter::updateDistribution()
    {
        switch (filter_)
        {
            case 0: distr_=updateGaussianDistribution(u_.size());
            break;
            case 1: distr_=averageDistribution(u_.size());
            break;
        }
    }

    dynamicgraph::Vector& Filter::getOutput(dynamicgraph::Vector& output, const int& time)
    {
        stateObservation::Vector lastInput=(inputSIN_.access(time));

        // Update the input window
        u_.push_front(lastInput);
        if(u_.size()>=n_) u_.pop_back();

        // Update the distribution accordingly
        updateDistribution();

        // Compute filtering
        output_.resize(lastInput.size()); output_.setZero();
        int i=0;
        for (iterator=u_.begin(); iterator != u_.end(); ++iterator)
        {
            output_+=distr_[i]*(*iterator);
            ++i;
        }

        output=(output_);
        return output;
    }
}


//
// Copyright (c) 2015,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-dynamic.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef STACKOFVECTORS_HH
#define STACKOFVECTORS_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geoemtry.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>


namespace sotStateObservation
{
    using dynamicgraph::Signal;
    using dynamicgraph::SignalPtr;
    using dynamicgraph::SignalTimeDependent;
    using dynamicgraph::Vector;
    using dynamicgraph::Matrix;
    using dynamicgraph::Entity;
    //using dynamicgraph::sot::VectorMultiBound;
    using dynamicgraph::sot::MatrixHomogeneous;
    using dynamicgraph::sot::MatrixRotation;
    using dynamicgraph::sot::VectorUTheta;
    //using dynamicgraph::sot::VectorRollPitchYaw;

    using namespace sotStateObservation;
    using namespace stateObservation;

        /**
           \brief
        */
        class StackOfContacts :
            public dynamicgraph::Entity,
            private boost::noncopyable //
        {
        public:
            /**
            \brief Constructor by name
            */
            StackOfContacts(const std::string& inName);

            ~StackOfContacts();

            /// Each entity should provide the name of the class it belongs to
            virtual const std::string& getClassName (void) const
            {
                return CLASS_NAME;
            }

            /// Header documentation of the python class
            virtual std::string getDocString () const
            {
                return
                    "Entity that compute the stack of contacts";
            }

            unsigned int& getNbSupport(unsigned int& , const int& time);

            Vector& getSupportPos1(Vector& , const int& time);
            MatrixHomogeneous& getHomoSupportPos1(MatrixHomogeneous& , const int& time);
            dynamicgraph::Vector& getForceSupport1(dynamicgraph::Vector& , const int& time);

            dynamicgraph::Vector& getSupportPos2(dynamicgraph::Vector& , const int& time);
            MatrixHomogeneous& getHomoSupportPos2(MatrixHomogeneous& , const int& time);
            dynamicgraph::Vector& getForceSupport2(dynamicgraph::Vector& , const int& time);


            /**
            \name Parameters
            @{
            */
        protected:
            /*
            \brief Class name
            */
            static const std::string CLASS_NAME;

        private:

            /// Methods
            void computeStack(const int& time);

            /// Signals
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> leftFootPositionSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceLeftFootSIN_;

            dynamicgraph::SignalPtr <MatrixHomogeneous, int> rightFootPositionSIN_;
            dynamicgraph::SignalPtr <Vector, int> forceRightFootSIN_;

            dynamicgraph::SignalPtr <unsigned int, int> nbSupportSOUT_;

            dynamicgraph::SignalPtr <Vector, int> supportPos1SOUT_;
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> homoSupportPos1SOUT_;
            dynamicgraph::SignalPtr <Vector, int> forceSupport1SOUT_;

            dynamicgraph::SignalPtr <Vector, int> supportPos2SOUT_;
            dynamicgraph::SignalPtr <MatrixHomogeneous, int> homoSupportPos2SOUT_;
            dynamicgraph::SignalPtr <Vector, int> forceSupport2SOUT_;

            /// Parameters
            double forceThreshold_, time_;
            unsigned int nbSupport_;
            dynamicgraph::Vector supportPos1_, supportPos2_, forceSupport1_, forceSupport2_;
            MatrixHomogeneous homoSupportPos1_, homoSupportPos2_;
      };

} // namespace sotStateObservation

#endif // STACKOFVECTORS_HH
